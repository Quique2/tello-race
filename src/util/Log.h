#pragma once
#include <cstdarg>
#include <string>

namespace logx {
enum Level { DEBUG = 0, INFO = 1, WARN = 2, ERR = 3 };
void setLevel(Level l);
Level level();
void setFile(const std::string& path);   // optional mirror of all messages to a file
void log(Level l, const char* fmt, ...);
double elapsed();                          // seconds since program start
}

#define LOG_DEBUG(...) ::logx::log(::logx::DEBUG, __VA_ARGS__)
#define LOG_INFO(...)  ::logx::log(::logx::INFO, __VA_ARGS__)
#define LOG_WARN(...)  ::logx::log(::logx::WARN, __VA_ARGS__)
#define LOG_ERROR(...) ::logx::log(::logx::ERR, __VA_ARGS__)
