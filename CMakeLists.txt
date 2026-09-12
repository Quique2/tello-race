cmake_minimum_required(VERSION 3.24)
project(tello_race VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# OpenCV (core, imgproc, imgcodecs, videoio(FFmpeg), highgui, calib3d)
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui calib3d)
message(STATUS "OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

# ---------------------------------------------------------------------------
# AprilTag (AprilRobotics) - fetched and built from source as a static library
# ---------------------------------------------------------------------------
include(FetchContent)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(apriltag
  GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
  GIT_TAG        v3.4.5
  GIT_SHALLOW    TRUE
  PATCH_COMMAND  ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/cmake/patch_apriltag.cmake <SOURCE_DIR>
  UPDATE_DISCONNECTED TRUE)
FetchContent_MakeAvailable(apriltag)
if(TARGET apriltag AND NOT MSVC)
  # third-party C code: keep the build quiet
  target_compile_options(apriltag PRIVATE -w)
endif()

# ---------------------------------------------------------------------------
# civetweb (embedded HTTP + WebSocket server for the ground-station web interface)
# ---------------------------------------------------------------------------
set(CIVETWEB_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_SERVER_EXECUTABLE OFF CACHE BOOL "" FORCE)
set(CIVETWEB_INSTALL_EXECUTABLE OFF CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_WEBSOCKETS ON CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_SSL OFF CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_ASAN OFF CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_CXX OFF CACHE BOOL "" FORCE)
set(CIVETWEB_DISABLE_CGI ON CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_IPV6 OFF CACHE BOOL "" FORCE)
FetchContent_Declare(civetweb
  GIT_REPOSITORY https://github.com/civetweb/civetweb.git
  GIT_TAG        v1.16
  GIT_SHALLOW    TRUE)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)   # civetweb's cmake_minimum_required is older than CMake 4 accepts
FetchContent_MakeAvailable(civetweb)
unset(CMAKE_POLICY_VERSION_MINIMUM)
if(TARGET civetweb-c-library AND NOT MSVC)
  target_compile_options(civetweb-c-library PRIVATE -w)
endif()

# ---------------------------------------------------------------------------
# tello_race executable
# ---------------------------------------------------------------------------
set(SRC
  src/main.cpp
  src/util/Config.cpp
  src/util/Log.cpp
  src/tello/UdpSocket.cpp
  src/tello/TelloState.cpp
  src/tello/IDrone.cpp
  src/tello/TelloDrone.cpp
  src/vision/FrameSource.cpp
  src/vision/CameraModel.cpp
  src/vision/TagDetector.cpp
  src/vision/LineDetector.cpp
  src/vision/Perception.cpp
  src/control/Navigator.cpp
  src/sim/Track.cpp
  src/sim/Simulator.cpp
  src/app/Mission.cpp
  src/app/Tools.cpp
  src/app/WebServer.cpp
  src/app/WebPage.cpp)

add_executable(tello_race ${SRC})
target_include_directories(tello_race PRIVATE src)
target_link_libraries(tello_race PRIVATE ${OpenCV_LIBS} apriltag civetweb-c-library)
target_include_directories(tello_race PRIVATE ${civetweb_SOURCE_DIR}/include)
if(WIN32)
  target_link_libraries(tello_race PRIVATE ws2_32)
  target_compile_definitions(tello_race PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
endif()
if(MSVC)
  target_compile_options(tello_race PRIVATE /W3 /utf-8)
else()
  target_compile_options(tello_race PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()
if(UNIX AND NOT APPLE)
  find_package(Threads REQUIRED)
  target_link_libraries(tello_race PRIVATE Threads::Threads)
endif()

# ---------------------------------------------------------------------------
# Runtime files next to the executable: config, OpenCV DLLs, MinGW runtime
# ---------------------------------------------------------------------------
add_custom_command(TARGET tello_race POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/config $<TARGET_FILE_DIR:tello_race>/config)

if(WIN32)
  add_custom_command(TARGET tello_race POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:tello_race> $<TARGET_FILE_DIR:tello_race>
    COMMAND_EXPAND_LISTS)
  # OpenCV's FFmpeg wrapper DLL (loaded at runtime by opencv_videoio, not linked)
  file(GLOB_RECURSE _ffmpeg_dlls "${OpenCV_INSTALL_PATH}/opencv_videoio_ffmpeg*.dll")
  if(NOT _ffmpeg_dlls)
    file(GLOB_RECURSE _ffmpeg_dlls "${OpenCV_DIR}/opencv_videoio_ffmpeg*.dll")
  endif()
  foreach(_dll ${_ffmpeg_dlls})
    add_custom_command(TARGET tello_race POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_dll}" $<TARGET_FILE_DIR:tello_race>)
  endforeach()
  # MinGW runtime DLLs (only needed for GCC builds)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    get_filename_component(_gcc_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
    foreach(_rt libstdc++-6.dll libgcc_s_seh-1.dll libwinpthread-1.dll)
      if(EXISTS "${_gcc_bin}/${_rt}")
        add_custom_command(TARGET tello_race POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_gcc_bin}/${_rt}" $<TARGET_FILE_DIR:tello_race>)
      endif()
    endforeach()
  endif()
endif()

install(TARGETS tello_race RUNTIME DESTINATION bin)
