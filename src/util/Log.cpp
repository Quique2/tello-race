#include "util/Log.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <mutex>

namespace logx {
namespace {
Level g_level = INFO;
std::mutex g_mtx;
std::ofstream g_file;
const auto g_start = std::chrono::steady_clock::now();
const char* names[] = {"DBG", "INF", "WRN", "ERR"};
}

void setLevel(Level l) { g_level = l; }
Level level() { return g_level; }
void setFile(const std::string& path) {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_file.close();
    g_file.open(path, std::ios::app);
}
double elapsed() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - g_start).count();
}

void log(Level l, const char* fmt, ...) {
    if (l < g_level) return;
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    std::lock_guard<std::mutex> lk(g_mtx);
    FILE* out = l >= WARN ? stderr : stdout;
    fprintf(out, "[%8.3f %s] %s\n", elapsed(), names[l], buf);
    fflush(out);
    if (g_file) g_file << "[" << elapsed() << " " << names[l] << "] " << buf << "\n" << std::flush;
}
}
