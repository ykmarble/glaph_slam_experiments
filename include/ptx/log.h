#ifndef LOG_H_
#define LOG_H_

#include <cstdio>
#include <cstdarg>

inline void ptx_log_impl(FILE* f, const char* level, const char* filename, int line, const char* msg, ...)
{
    fprintf(f, "[%s] ", level);

    va_list args;
    va_start(args, msg);
    vfprintf(f, msg, args);
    va_end(args);

    fprintf(f, " (%s:%d)\n", filename, line);
}

#define LOG_DEBUG(...) ptx_log_impl(stdout, "DEBUG", __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...) ptx_log_impl(stdout, " INFO", __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...) ptx_log_impl(stderr, " WARN", __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) ptx_log_impl(stderr, "ERROR", __FILE__, __LINE__, __VA_ARGS__)
#define LOG_FATAL(...) ptx_log_impl(stderr, "FATAL", __FILE__, __LINE__, __VA_ARGS__)

#endif  //LOG_HPP_
