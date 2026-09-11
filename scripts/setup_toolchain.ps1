# Recreates the portable (no admin) C++ toolchain used by this project in C:\dev:
#   winlibs GCC 16.2 (UCRT, POSIX threads), CMake, Ninja, and OpenCV 4.14.0 built from source with FFmpeg.
# Usage:  powershell -ExecutionPolicy Bypass -File scripts\setup_toolchain.ps1
# Takes ~10-15 minutes. Re-runnable: finished steps are skipped.
$ErrorActionPreference = "Stop"
$dev = "C:\dev"; $dl = "$dev\downloads"
New-Item -ItemType Directory -Force -Path $dl, "$dev\src", "$dev\build" | Out-Null
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$gccTag = "16.2.0posix-14.0.0-ucrt-r1"
$gccZip = "winlibs-x86_64-posix-seh-gcc-16.2.0-mingw-w64ucrt-14.0.0-r1.zip"
$cmakeVer = "4.4.3"; $ninjaVer = "1.13.2"; $opencvVer = "4.14.0"

function Get-File($url, $out) {
    if (-not (Test-Path $out)) { Write-Host "downloading $url"; curl.exe -L --retry 5 -o $out $url; if ($LASTEXITCODE -ne 0) { throw "download failed: $url" } }
}
Get-File "https://github.com/brechtsanders/winlibs_mingw/releases/download/$gccTag/$gccZip" "$dl\$gccZip"
Get-File "https://github.com/Kitware/CMake/releases/download/v$cmakeVer/cmake-$cmakeVer-windows-x86_64.zip" "$dl\cmake.zip"
Get-File "https://github.com/ninja-build/ninja/releases/download/v$ninjaVer/ninja-win.zip" "$dl\ninja.zip"

if (-not (Test-Path "$dev\mingw64\bin\gcc.exe")) { Expand-Archive "$dl\$gccZip" -DestinationPath $dev }
if (-not (Test-Path "$dev\cmake\bin\cmake.exe")) { Expand-Archive "$dl\cmake.zip" -DestinationPath $dev; Rename-Item "$dev\cmake-$cmakeVer-windows-x86_64" "$dev\cmake" }
if (-not (Test-Path "$dev\ninja\ninja.exe")) { Expand-Archive "$dl\ninja.zip" -DestinationPath "$dev\ninja" }
$env:Path = "$dev\mingw64\bin;$dev\cmake\bin;$dev\ninja;" + $env:Path

if (-not (Test-Path "$dev\src\opencv\.git")) { git clone --depth 1 --branch $opencvVer https://github.com/opencv/opencv.git "$dev\src\opencv" }

# OpenCV's FFmpeg wrapper is normally downloaded by CMake; that download fails TLS validation on some PCs,
# so fetch the three files with curl into OpenCV's cache (names = <md5>-<file>, hashes from 3rdparty/ffmpeg/ffmpeg.cmake).
$ffcmake = Get-Content "$dev\src\opencv\3rdparty\ffmpeg\ffmpeg.cmake" -Raw
$commit = [regex]::Match($ffcmake, 'FFMPEG_BINARIES_COMMIT "([0-9a-f]+)"').Groups[1].Value
$h32 = [regex]::Match($ffcmake, 'FFMPEG_FILE_HASH_BIN32 "([0-9a-f]+)"').Groups[1].Value
$h64 = [regex]::Match($ffcmake, 'FFMPEG_FILE_HASH_BIN64 "([0-9a-f]+)"').Groups[1].Value
$hcm = [regex]::Match($ffcmake, 'FFMPEG_FILE_HASH_CMAKE "([0-9a-f]+)"').Groups[1].Value
$cache = "$dev\src\opencv\.cache\ffmpeg"; New-Item -ItemType Directory -Force -Path $cache | Out-Null
$base = "https://raw.githubusercontent.com/opencv/opencv_3rdparty/$commit/ffmpeg"
foreach ($pair in @(@($h32, "opencv_videoio_ffmpeg.dll"), @($h64, "opencv_videoio_ffmpeg_64.dll"), @($hcm, "ffmpeg_version.cmake"))) {
    $target = "$cache\$($pair[0])-$($pair[1])"
    if (-not (Test-Path $target) -or (Get-Item $target).Length -eq 0) { curl.exe -L --retry 5 -o $target "$base/$($pair[1])" }
}

if (-not (Test-Path "$dev\opencv\OpenCVConfig.cmake")) {
    cmake -S "$dev\src\opencv" -B "$dev\build\opencv" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dev\opencv" `
        -DCMAKE_C_COMPILER="$dev\mingw64\bin\gcc.exe" -DCMAKE_CXX_COMPILER="$dev\mingw64\bin\g++.exe" `
        -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,calib3d -DBUILD_SHARED_LIBS=ON `
        -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF -DBUILD_opencv_apps=OFF `
        -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF -DWITH_FFMPEG=ON -DWITH_DSHOW=ON -DWITH_OPENEXR=OFF -DWITH_JASPER=OFF `
        -DWITH_OPENJPEG=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_IPP=OFF -DWITH_OPENCL=OFF -DWITH_CUDA=OFF -DWITH_PROTOBUF=OFF `
        -DWITH_ADE=OFF -DWITH_QUIRC=OFF -DWITH_OBSENSOR=OFF -DWITH_ITT=OFF -DWITH_DIRECTX=OFF -DWITH_VTK=OFF -DWITH_EIGEN=OFF `
        -DWITH_LAPACK=OFF -DWITH_OPENMP=OFF -DWITH_TBB=OFF -DWITH_1394=OFF -DWITH_GSTREAMER=OFF -DWITH_DIRECTML=OFF `
        -DBUILD_ZLIB=ON -DBUILD_PNG=ON -DBUILD_JPEG=ON -DOPENCV_GENERATE_PKGCONFIG=OFF -DENABLE_PRECOMPILED_HEADERS=OFF
    if ($LASTEXITCODE -ne 0) { throw "OpenCV configure failed" }
    cmake --build "$dev\build\opencv" --target install
    if ($LASTEXITCODE -ne 0) { throw "OpenCV build failed" }
}
Write-Host "`nToolchain ready in $dev. Now run scripts\build.ps1" -ForegroundColor Green
