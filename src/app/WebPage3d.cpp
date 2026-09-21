// 3D view of a simulation episode, served at /sim3d: the rendered floor as a texture, the track and tags, a Tello
// model at the true pose, its camera footprint on the floor and the flown trail. Pose arrives over the same
// websocket as the main page (state JSON "sim" block); the scene comes from /sim/scene.json and /sim/world.png.
#include "app/WebServer.h"

static const char* kSim3dHtml = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><title>tello_race sim 3D</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
html,body{margin:0;height:100%;background:#15171c;color:#dfe3ea;font:14px system-ui,Segoe UI,Roboto,sans-serif;overflow:hidden}
#hud{position:fixed;left:12px;top:10px;background:rgba(20,22,28,.75);padding:8px 12px;border-radius:8px;line-height:1.5;pointer-events:none}
#hud b{color:#ffd166}
#ctl{position:fixed;right:12px;top:10px;background:rgba(20,22,28,.75);padding:8px 12px;border-radius:8px}
#ctl label{display:block;margin:2px 0}
#msg{position:fixed;left:12px;bottom:10px;color:#ff7b7b}
canvas{display:block}
</style>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
</head><body>
<div id="hud">connecting…</div>
<div id="ctl"><label><input type="checkbox" id="follow" checked> follow the drone</label><label><input type="checkbox" id="foot" checked> camera footprint</label><label><input type="checkbox" id="trail" checked> trail</label>
<hr style="border:0;border-top:1px solid #444;margin:8px 0">
<div style="font-weight:600;margin-bottom:4px">simulation settings</div>
<label>camera view <select id="pitch"><option value="12">12° stock camera (fly at 50 cm)</option><option value="30">30° (mirror)</option><option value="45">45°</option><option value="67">67°</option><option value="68" selected>68° your holder</option><option value="70">70°</option><option value="74">74°</option><option value="83">83°</option><option value="96">96° old holder</option><option value="108">108°</option></select></label>
<label>altitude <select id="alt"><option value="50">50 cm</option><option value="100">100 cm</option><option value="150" selected>150 cm</option><option value="200">200 cm</option><option value="250">250 cm</option></select></label>
<label>course <select id="track"><option value="example">example (slides)</option><option value="random" selected>random</option></select></label>
<label>seed <input id="seed" type="number" value="300" min="0" style="width:80px"> <span id="seedhint" style="color:#8a94a6">300-319 = the tested set</span></label>
<div style="margin-top:6px"><button id="restart">restart with these settings</button></div>
<div id="cur" style="color:#8a94a6;margin-top:4px"></div></div>
<div id="msg"></div>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id);
let tok=localStorage.getItem('tello_token')||'';const qtok=new URLSearchParams(location.search).get('token');
if(qtok){tok=qtok;localStorage.setItem('tello_token',tok);history.replaceState(null,'',location.pathname);}
const q=tok?('?token='+encodeURIComponent(tok)):'';
const W=v=>new THREE.Vector3(v[0],v[2]||0,-v[1]);           // sim world (x, y, z-up) -> three (X, Y-up, Z)

const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(devicePixelRatio);renderer.setSize(innerWidth,innerHeight);document.body.appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color(0x15171c);
const cam=new THREE.PerspectiveCamera(50,innerWidth/innerHeight,0.02,200);cam.position.set(3,3.5,4);
const ctrl=new OrbitControls(cam,renderer.domElement);ctrl.target.set(0,0,0);ctrl.enableDamping=true;
scene.add(new THREE.HemisphereLight(0xffffff,0x334455,1.1));const sun=new THREE.DirectionalLight(0xffffff,1.2);sun.position.set(4,8,3);scene.add(sun);
addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);});

let sc=null;
try{sc=await (await fetch('/sim/scene.json'+q)).json();}catch(e){$('msg').textContent='no simulation scene: run "tello_race sim --set web.enabled=1" (with the token in the URL)';}
if(sc){
  $('cur').textContent=`now: ${sc.pitch.toFixed(0)}° at ${sc.alt} cm, ${sc.track} seed ${sc.seed}`;
  const setSel=(id,v)=>{const e=$(id);if([...e.options].some(o=>o.value==String(v)))e.value=String(v);};
  setSel('pitch',Math.round(sc.pitch));setSel('alt',sc.alt);setSel('track',sc.track);$('seed').value=sc.seed;
  const tex=new THREE.TextureLoader().load('/sim/world.png'+q);tex.colorSpace=THREE.SRGBColorSpace;tex.anisotropy=8;
  const floor=new THREE.Mesh(new THREE.PlaneGeometry(sc.worldSize,sc.worldSize),new THREE.MeshLambertMaterial({map:tex}));
  floor.rotation.x=-Math.PI/2;scene.add(floor);
  const half=sc.worldSize/2-0.3;                                       // the net (sim.bounds_margin_m)
  const net=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(2*half,2.6,2*half)),new THREE.LineBasicMaterial({color:0xff5555,transparent:true,opacity:0.5}));
  net.position.y=1.3;scene.add(net);
  for(const seg of sc.segments){                                        // track polylines, slightly above the floor
    const g=new THREE.BufferGeometry().setFromPoints(seg.map(p=>W([p[0],p[1],0.004])));
    scene.add(new THREE.Line(g,new THREE.LineBasicMaterial({color:0x66ccff})));
  }
  for(const t of sc.tags){                                              // tag posts with the id
    const post=new THREE.Mesh(new THREE.CylinderGeometry(0.01,0.01,0.25,8),new THREE.MeshLambertMaterial({color:t.id==4?0xff4444:0xffd166}));
    post.position.copy(W([t.x,t.y,0.125]));scene.add(post);
    const c=document.createElement('canvas');c.width=64;c.height=64;const g=c.getContext('2d');g.fillStyle='#fff';g.fillRect(0,0,64,64);g.fillStyle='#000';g.font='bold 40px sans-serif';g.textAlign='center';g.fillText('ID'+t.id,32,46);
    const sp=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(c)}));sp.scale.set(0.22,0.22,1);sp.position.copy(W([t.x,t.y,0.32]));scene.add(sp);
  }
}

// --- Tello model (metres): 98 x 92.5 x 41 mm, forward = +X of the group
const tello=new THREE.Group();
{
  const grey=new THREE.MeshStandardMaterial({color:0xd8d8d8,roughness:0.6}),dark=new THREE.MeshStandardMaterial({color:0x333333,roughness:0.8});
  const body=new THREE.Mesh(new THREE.BoxGeometry(0.075,0.028,0.032),grey);body.position.y=0.012;tello.add(body);
  const nose=new THREE.Mesh(new THREE.BoxGeometry(0.02,0.02,0.026),dark);nose.position.set(0.045,0.012,0);tello.add(nose);
  const lens=new THREE.Mesh(new THREE.CylinderGeometry(0.004,0.004,0.004,12),new THREE.MeshStandardMaterial({color:0x2266ff}));lens.rotation.z=Math.PI/2;lens.position.set(0.056,0.012,0);tello.add(lens);
  for(const [sx,sz] of [[1,1],[1,-1],[-1,1],[-1,-1]]){
    const arm=new THREE.Mesh(new THREE.BoxGeometry(0.05,0.008,0.008),grey);arm.position.set(sx*0.028,0.006,sz*0.028);arm.rotation.y=-sx*sz*Math.PI/4;tello.add(arm);
    const motor=new THREE.Mesh(new THREE.CylinderGeometry(0.007,0.007,0.014,12),dark);motor.position.set(sx*0.046,0.012,sz*0.046);tello.add(motor);
    const prop=new THREE.Mesh(new THREE.CylinderGeometry(0.038,0.038,0.001,24),new THREE.MeshStandardMaterial({color:0x99aacc,transparent:true,opacity:0.35}));prop.position.set(sx*0.046,0.021,sz*0.046);tello.add(prop);
    const guard=new THREE.Mesh(new THREE.TorusGeometry(0.04,0.002,6,32),grey);guard.rotation.x=Math.PI/2;guard.position.set(sx*0.046,0.016,sz*0.046);tello.add(guard);
  }
  const mirror=new THREE.Mesh(new THREE.BoxGeometry(0.02,0.0015,0.028),new THREE.MeshStandardMaterial({color:0x88ddff,metalness:0.9,roughness:0.1}));
  mirror.position.set(0.07,0.004,0);mirror.rotation.z=-0.63;tello.add(mirror);          // 36 deg seat in front of the lens
}
scene.add(tello);
const shadow=new THREE.Mesh(new THREE.CircleGeometry(0.06,24),new THREE.MeshBasicMaterial({color:0x000000,transparent:true,opacity:0.35}));shadow.rotation.x=-Math.PI/2;shadow.position.y=0.003;scene.add(shadow);

// camera footprint on the floor + rays; trail
const footMat=new THREE.MeshBasicMaterial({color:0xffd166,transparent:true,opacity:0.22,side:THREE.DoubleSide,depthWrite:false});
const foot=new THREE.Mesh(new THREE.BufferGeometry(),footMat);scene.add(foot);
const rays=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xffd166,transparent:true,opacity:0.6}));scene.add(rays);
const trailPts=[];const trail=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xff9f43}));scene.add(trail);

function footprint(x,y,z,yawDeg,pitchDeg){
  const h=THREE.MathUtils.degToRad(sc?sc.hfov:55)/2,v=THREE.MathUtils.degToRad(sc?sc.vfov:43)/2,p=THREE.MathUtils.degToRad(pitchDeg),psi=THREE.MathUtils.degToRad(yawDeg);
  const pts=[];
  for(const [sa,se] of [[-1,1],[1,1],[1,-1],[-1,-1]]){                 // corners: top-left, top-right, bottom-right, bottom-left
    const e=-p+se*v,a=sa*h;                                            // elevation (down negative), azimuth (left positive)
    let dx=Math.cos(e)*Math.cos(a),dy=Math.cos(e)*Math.sin(a),dz=Math.sin(e);
    const wx=dx*Math.cos(psi)-dy*Math.sin(psi),wy=dx*Math.sin(psi)+dy*Math.cos(psi);
    let t=dz<-1e-3?-z/dz:8/Math.hypot(wx,wy);t=Math.min(t,8);
    pts.push([x+t*wx,y+t*wy,0.006]);
  }
  return pts;
}
let pose=null;
function setPose(s){
  pose=s;tello.position.copy(W([s.x,s.y,s.z]));tello.rotation.y=THREE.MathUtils.degToRad(s.yaw);
  shadow.position.set(tello.position.x,0.003,tello.position.z);
  if($('foot').checked){
    const c=footprint(s.x,s.y,s.z,s.yaw,s.camPitch).map(W);
    foot.geometry.dispose();foot.geometry=new THREE.BufferGeometry().setFromPoints([c[0],c[1],c[2],c[0],c[2],c[3]]);foot.visible=true;
    const o=W([s.x,s.y,s.z]);const rp=[];for(const k of c){rp.push(o,k);}rays.geometry.dispose();rays.geometry=new THREE.BufferGeometry().setFromPoints(rp);rays.visible=true;
  } else {foot.visible=false;rays.visible=false;}
  if($('trail').checked&&s.z>0.05){const p=W([s.x,s.y,Math.max(0.01,s.z)]);if(!trailPts.length||trailPts[trailPts.length-1].distanceTo(p)>0.03){trailPts.push(p);if(trailPts.length>3000)trailPts.shift();trail.geometry.dispose();trail.geometry=new THREE.BufferGeometry().setFromPoints(trailPts);}}
}
let ws,phase='',seg=0,t=0,fps=0,role='';
$('restart').onclick=()=>{
  if(!ws||ws.readyState!==1){$('msg').textContent='not connected';return;}
  if(role!=='operator'){$('msg').textContent='changing the simulation needs the operator token in the URL (?token=...)';return;}
  const arg=`pitch=${$('pitch').value};alt=${$('alt').value};track=${$('track').value};seed=${$('seed').value}`;
  ws.send(JSON.stringify({cmd:'sim',arg}));$('msg').textContent='restarting: '+arg;trailPts.length=0;
  setTimeout(()=>location.reload(),2500);                                // the new course has a new floor and tags
};
function connect(){
  ws=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws'+q);
  ws.onmessage=ev=>{let m;try{m=JSON.parse(ev.data);}catch(e){return;}
    if(m.role)role=m.role;
    if(m.phase!==undefined){phase=m.phase;seg=m.seg;t=m.t;fps=m.fps;}
    if(m.sim){if(phase==='TAKEOFF'&&trailPts.length>50&&t<1)trailPts.length=0;setPose(m.sim);}
    if(m.error)$('msg').textContent=m.error;};
  ws.onclose=()=>setTimeout(connect,1500);
}
connect();
function loop(){
  if(pose&&$('follow').checked){ctrl.target.lerp(tello.position,0.15);}
  ctrl.update();renderer.render(scene,cam);
  if(pose)$('hud').innerHTML=`<b>${phase}</b> seg ${seg} t=${t.toFixed(1)}s &nbsp; x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} z=${pose.z.toFixed(2)} yaw=${pose.yaw.toFixed(0)}° &nbsp; camera ${pose.camPitch.toFixed(0)}° down &nbsp; ${fps.toFixed(0)} fps`;
  requestAnimationFrame(loop);
}
loop();
</script></body></html>
)HTML";

const char* sim3dPageHtml() { return kSim3dHtml; }
