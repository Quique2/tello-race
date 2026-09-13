# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "shapely>=2.0", "trimesh", "networkx", "scipy", "rtree",
#                 "mapbox-earcut", "manifold3d", "opencv-python-headless>=4.8", "pillow"]
# ///
"""
AprilTag plates (family tag36h11) for 3D printing, made for the tello_race detector.

  uv run genera_apriltags.py                        -> IDs 1..4, black square 160 mm (= tag.size_m 0.16)
  uv run genera_apriltags.py --ids 1 2 --black 120  -> smaller tags (then set tag.size_m = 0.12)
  uv run genera_apriltags.py --video test.mp4       -> also a synthetic video to feed "tello_race replay"

Same recipe as Prints-MIND/genera_qr_mind.py: flush two-colour inlay (base cream + dark modules, 0.4 mm colour
layer, the most legible geometry in FDM), <name>.3mf with two extruders + <name>_base.stl / <name>_tag.stl, and a
300 dpi PNG to print on paper at exact scale as a fallback.
Layout: 8x8 tag cells (1-cell black border + 6x6 data) inside a 1-cell white quiet zone -> plate = 10 cells.
Self-test: the rendered face (navy on cream, the pessimistic colour pair) must decode with OpenCV's tag36h11
dictionary to the same ID before anything is written.
"""
import argparse, os, zipfile
import numpy as np, cv2, trimesh
from trimesh.creation import box as tbox, extrude_polygon
from shapely.geometry import box as sbox

CREAM_RGB, DARK_RGB = (245, 239, 216), (0, 0, 128)   # cream #F5EFD8, navy #000080; black filament is even better
N = 8                                                # tag36h11: 6x6 data + 1-cell black border


def tag_matrix(tag_id):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = cv2.aruco.generateImageMarker(d, tag_id, N * 10, 1)        # 8 cells x 10 px, border 1 cell
    return img.reshape(N, 10, N, 10).mean(axis=(1, 3)) < 128           # True = dark, [row, col], row 0 = top


def render_face(M, border, px):
    total = N + 2 * border
    img = np.zeros((total * px, total * px, 3), np.uint8)
    img[:] = CREAM_RGB[::-1]                                          # BGR
    for r in range(N):
        for c in range(N):
            if M[r, c]:
                img[(border + r) * px:(border + r + 1) * px, (border + c) * px:(border + c + 1) * px] = DARK_RGB[::-1]
    return img


def selftest(face, tag_id):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    _, ids, _ = det.detectMarkers(cv2.cvtColor(face, cv2.COLOR_BGR2GRAY))
    found = [] if ids is None else [int(i) for i in ids.flatten()]
    assert found == [tag_id], f"self-test failed for id {tag_id}: detected {found}"


def write_3mf(path, title, base_all, dark, center):
    def mesh_xml(oid, m):
        v = "\n".join(f'     <vertex x="{x:.5f}" y="{y:.5f}" z="{z:.5f}"/>' for x, y, z in m.vertices)
        t = "\n".join(f'     <triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in m.faces)
        return (f'  <object id="{oid}" p:UUID="{oid:08d}-81cb-4c03-9d28-80fed5dfa1dc" type="model">\n'
                '   <mesh>\n    <vertices>\n' + v + '\n    </vertices>\n'
                '    <triangles>\n' + t + '\n    </triangles>\n   </mesh>\n  </object>\n')
    ns = ('xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
          'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
          'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p"')
    tf = f"1 0 0 0 1 0 0 0 1 {center:.1f} {center:.1f} 0"
    obj_file = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {ns}>\n'
                ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n <resources>\n'
                + mesh_xml(11, base_all) + mesh_xml(12, dark) + ' </resources>\n <build/>\n</model>\n')
    root = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {ns}>\n'
            ' <metadata name="Application">OrcaSlicer-compatible</metadata>\n'
            ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n'
            f' <metadata name="Title">{title}</metadata>\n <resources>\n'
            '  <object id="10" p:UUID="00000010-61cb-4c03-9d28-80fed5dfa1dc" type="model">\n   <components>\n'
            '    <component p:path="/3D/Objects/object_10.model" objectid="11" '
            'p:UUID="00000011-b206-40ff-9872-83e8017abed1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>\n'
            '    <component p:path="/3D/Objects/object_10.model" objectid="12" '
            'p:UUID="00000012-b206-40ff-9872-83e8017abed1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>\n'
            '   </components>\n  </object>\n </resources>\n'
            ' <build p:UUID="2c7c17d8-22b5-4d84-8835-1976022ea372">\n'
            f'  <item objectid="10" p:UUID="00000010-b1ec-4553-aec9-835e5b724bb4" transform="{tf}" printable="1"/>\n'
            ' </build>\n</model>\n')
    ms = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n  <object id="10">\n'
          f'    <metadata key="name" value="{title}"/>\n    <metadata key="extruder" value="1"/>\n'
          '    <part id="11" subtype="normal_part">\n      <metadata key="name" value="base (crema)"/>\n'
          '      <metadata key="extruder" value="1"/>\n'
          '      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
          '      <mesh_stat edges_fixed="0" degenerate_facets="0" facets_removed="0" facets_reversed="0" backwards_edges="0"/>\n'
          '    </part>\n    <part id="12" subtype="normal_part">\n      <metadata key="name" value="modulos tag (oscuro)"/>\n'
          '      <metadata key="extruder" value="2"/>\n'
          '      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
          '      <mesh_stat edges_fixed="0" degenerate_facets="0" facets_removed="0" facets_reversed="0" backwards_edges="0"/>\n'
          '    </part>\n  </object>\n  <plate>\n    <metadata key="plater_id" value="1"/>\n'
          '    <metadata key="plater_name" value=""/>\n    <metadata key="locked" value="false"/>\n'
          '    <model_instance>\n      <metadata key="object_id" value="10"/>\n      <metadata key="instance_id" value="0"/>\n'
          '      <metadata key="identify_id" value="100"/>\n    </model_instance>\n  </plate>\n'
          f'  <assemble>\n   <assemble_item object_id="10" instance_id="0" transform="{tf}" offset="0 0 0" />\n'
          '  </assemble>\n</config>\n')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?>\n'
                   '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">\n'
                   ' <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>\n'
                   ' <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>\n</Types>')
        z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?>\n'
                   '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
                   ' <Relationship Target="/3D/3dmodel.model" Id="rel-1" '
                   'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>')
        z.writestr("3D/3dmodel.model", root)
        z.writestr("3D/_rels/3dmodel.model.rels", '<?xml version="1.0" encoding="UTF-8"?>\n'
                   '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
                   ' <Relationship Target="/3D/Objects/object_10.model" Id="rel-10" '
                   'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>')
        z.writestr("3D/Objects/object_10.model", obj_file)
        z.writestr("Metadata/model_settings.config", ms)


def build(tag_id, black_mm, border, th, top, outdir):
    M = tag_matrix(tag_id)
    cell = black_mm / N
    tile = (N + 2 * border) * cell
    face = render_face(M, border, px=40)
    selftest(face, tag_id)                                             # before writing anything
    EPS = 0.04                                                         # corner-touching boxes need a tiny overlap

    def cell_box(r, c):
        b = tbox(extents=[cell + EPS, cell + EPS, top])
        b.apply_translation([(border + c + 0.5) * cell - tile / 2, tile / 2 - (border + r + 0.5) * cell, th - top / 2])
        return b

    dark = [cell_box(r, c) for r in range(N) for c in range(N) if M[r, c]]
    light = [cell_box(r, c) for r in range(N) for c in range(N) if not M[r, c]]
    B = border * cell
    for w, h, cx, cy in [(tile, B, 0, tile / 2 - B / 2), (tile, B, 0, -tile / 2 + B / 2),
                         (B, tile - 2 * B, -tile / 2 + B / 2, 0), (B, tile - 2 * B, tile / 2 - B / 2, 0)]:
        b = tbox(extents=[w + EPS, h + EPS, top])
        b.apply_translation([cx, cy, th - top / 2])
        light.append(b)
    base = extrude_polygon(sbox(-tile / 2, -tile / 2, tile / 2, tile / 2), th - top)
    light_m, dark_m = trimesh.util.concatenate(light), trimesh.util.concatenate(dark)
    for m in (base, light_m, dark_m):
        assert m.is_watertight
    base_all = trimesh.util.concatenate([base, light_m])

    name = f"apriltag36h11_id{tag_id}_{int(round(black_mm))}mm"
    base_all.export(os.path.join(outdir, f"{name}_base.stl"))
    dark_m.export(os.path.join(outdir, f"{name}_tag.stl"))
    write_3mf(os.path.join(outdir, f"{name}.3mf"), f"AprilTag 36h11 id {tag_id} ({int(round(black_mm))} mm)",
              base_all, dark_m, center=tile / 2 + 10)
    paper = render_face(M, border, px=int(round(cell / 25.4 * 300)))   # 300 dpi, exact scale
    cv2.putText(paper, f"tag36h11 id {tag_id}  black square {black_mm:.0f} mm  (print at 100 %)",
                (10, paper.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (60, 60, 60), 2)
    cv2.imwrite(os.path.join(outdir, f"{name}_paper_300dpi.png"), paper)
    print(f"id {tag_id}: plate {tile:.0f} x {tile:.0f} x {th:.1f} mm, cell {cell:.1f} mm, "
          f"{int(M.sum())} dark cells, self-test OK -> {name}.3mf / _base.stl / _tag.stl / _paper_300dpi.png")
    return face


def write_video(faces, path, seconds=3, fps=24):
    """Synthetic camera view (960x720): each tag plate ~320 px wide in the lower half of a grey floor."""
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (960, 720))
    for tag_id, face in faces:
        small = cv2.resize(face, (320, 320), interpolation=cv2.INTER_AREA)
        for k in range(seconds * fps):
            frame = np.full((720, 960, 3), 120, np.uint8)
            noise = np.random.default_rng(k).integers(-6, 7, frame.shape, dtype=np.int16)
            frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
            x = 320 + int(40 * np.sin(k / 12.0))
            frame[330:650, x:x + 320] = small
            vw.write(frame)
    vw.release()
    print(f"test video -> {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--black", type=float, default=160.0, help="black square edge in mm (tag.size_m * 1000)")
    ap.add_argument("--border", type=int, default=1, help="white quiet zone in cells (1 minimum)")
    ap.add_argument("--th", type=float, default=1.6, help="plate thickness incl. the colour layer, mm")
    ap.add_argument("--top", type=float, default=0.4, help="colour layer thickness, mm")
    ap.add_argument("--outdir", default=".")
    ap.add_argument("--video", default="", help="also write a synthetic test video for tello_race replay")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    faces = [(i, build(i, a.black, a.border, a.th, a.top, a.outdir)) for i in a.ids]
    if a.video:
        write_video(faces, a.video)
    print("Filament 1 = cream (or white), 2 = black (navy also decodes). Matte PLA. Keep the quiet zone clean.")
