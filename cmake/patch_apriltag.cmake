# Make apriltag's Windows pthread shim MSVC-only so MinGW builds use the real winpthreads consistently
# (otherwise C++ translation units that include <mutex>/<thread> see conflicting pthread typedefs).
set(SRC "${CMAKE_ARGV3}")
foreach(f "common/pthreads_cross.h" "common/pthreads_cross.c")
  set(path "${SRC}/${f}")
  if(EXISTS "${path}")
    file(READ "${path}" content)
    string(REPLACE "#ifdef _WIN32" "#if defined(_WIN32) && !defined(__MINGW32__)" content "${content}")
    # MinGW has no sysconf(): use the Win32 API for the processor count
    string(REPLACE "    return (unsigned int)sysconf(_SC_NPROCESSORS_ONLN);"
                   "#ifdef __MINGW32__\n    SYSTEM_INFO si; GetSystemInfo(&si); return (unsigned int)si.dwNumberOfProcessors;\n#else\n    return (unsigned int)sysconf(_SC_NPROCESSORS_ONLN);\n#endif"
                   content "${content}")
    string(REPLACE "#include <unistd.h>" "#include <unistd.h>\n#ifdef __MINGW32__\n#include <windows.h>\n#endif" content "${content}")
    file(WRITE "${path}" "${content}")
    message(STATUS "patched ${f}")
  endif()
endforeach()
