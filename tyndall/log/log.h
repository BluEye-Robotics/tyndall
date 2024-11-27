#pragma once

/*
  log.h
  log_error, log_warning etc. follows printf syntax.
  Format strings follows fmtlib format if LOG_FMT is defined,
  It uses printf format instead if LOG_PRINTF is defined.
*/

#ifdef __cplusplus
#include <cassert>
#include <cstring>
#include <string>
#include <utility>
#else
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

// interface
#define log_log(lvl, fmt, ...) log_cat_log("", lvl, fmt, __VA_ARGS__)
#define log_error(...) log_log(log_level_error, __VA_ARGS__)
#define log_warning(...) log_log(log_level_warning, __VA_ARGS__)
#define log_info(...) log_log(log_level_info, __VA_ARGS__)
#define log_debug(...) log_log(log_level_debug, __VA_ARGS__)

#define log_cat_log(cat, lvl, fmt, ...) log_impl(fmt, lvl, cat, __VA_ARGS__)
#define log_cat_error(cat, ...) log_cat_log(cat, log_level_error, __VA_ARGS__)
#define log_cat_warning(cat, ...)                                              \
  log_cat_log(cat, log_level_warning, __VA_ARGS__)
#define log_cat_info(cat, ...) log_cat_log(cat, log_level_info, __VA_ARGS__)
#define log_cat_debug(cat, ...) log_cat_log(cat, log_level_debug, __VA_ARGS__)

#define log_init(...)                                                          \
  do {                                                                         \
  } while (0)
#define log_str log_str_impl
#define log_flush log_flush_impl
#define log_level log_level_impl

#define log_once(log_func, ...)                                                \
  do {                                                                         \
    static int run = 1;                                                        \
    if (run) {                                                                 \
      log_func(__VA_ARGS__);                                                   \
      run = 0;                                                                 \
    }                                                                          \
  } while (0)

#define log_once_error(...) log_once(log_error, __VA_ARGS__)
#define log_once_warning(...) log_once(log_warning, __VA_ARGS__)
#define log_once_info(...) log_once(log_info, __VA_ARGS__)
#define log_once_debug(...) log_once(log_debug, __VA_ARGS__)

#define log_cat_once_error(...) log_once(log_cat_error, __VA_ARGS__)
#define log_cat_once_warning(...) log_once(log_cat_warning, __VA_ARGS__)
#define log_cat_once_info(...) log_once(log_cat_info, __VA_ARGS__)
#define log_cat_once_debug(...) log_once(log_cat_debug, __VA_ARGS__)

#define log_errno(fmt, ...)                                                    \
  log_error("[" log_color_red log_color_bold "errno" log_color_reset           \
            ": " LOG_PATTERN_STR "] " fmt,                                     \
            strerror(errno) __VA_OPT__(, ) __VA_ARGS__)

#define log_flushed_error(...)                                                 \
  do {                                                                         \
    log_error(__VA_ARGS__);                                                    \
    log_flush();                                                               \
  } while (0)
#define log_flushed_warning(...)                                               \
  do {                                                                         \
    log_warning(__VA_ARGS__);                                                  \
    log_flush();                                                               \
  } while (0)
#define log_flushed_info(...)                                                  \
  do {                                                                         \
    log_info(__VA_ARGS__);                                                     \
    log_flush();                                                               \
  } while (0)
#define log_flushed_debug(...)                                                 \
  do {                                                                         \
    log_debug(__VA_ARGS__);                                                    \
    log_flush();                                                               \
  } while (0)

// colors
#define log_color_red "\033[31m"
#define log_color_green "\033[32m"
#define log_color_yellow "\033[33m"
#define log_color_blue "\033[34m"
#define log_color_magenta "\033[35m"
#define log_color_cyan "\033[36m"
#define log_color_reset "\033[0m"
#define log_color_bold "\033[1m"

// implementation

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  log_level_debug,
  log_level_info,
  log_level_warning,
  log_level_error
} log_level_t;

const char *to_color_string(log_level_t lvl);

/*
  Reads the log level from the LOG_LEVEL environment variable.
  Optional global / fallback level first, then key-values of categories and
  debug level. Default category is function name. Examples: LOG_LEVEL=error
  ./my_program LOG_LEVEL=error,my_category:debug ./my_program
    LOG_LEVEL=my_ca*e*ry:info ./my_program
*/
log_level_t log_level(const char *cat);
#ifdef __cplusplus
} // extern "C"
#endif

#define log_impl(fmt, lvl, category, ...)                                      \
  do {                                                                         \
    const char *cat =                                                          \
        (const char *)((category && strlen(category)) ? category               \
                                                      : __FUNCTION__);         \
                                                                               \
    log_format_impl(fmt, lvl, cat, __VA_ARGS__);                               \
  } while (0)

#ifdef __cplusplus
extern "C" {
#endif

#define log_str_impl(str, lvl, cat)                                            \
  do {                                                                         \
    if (lvl >= log_level(cat))                                                 \
      printf("[%s %s:%d " log_color_magenta "%s" log_color_reset "] %s",       \
             to_color_string(lvl), __FILE__, __LINE__, cat, str);              \
  } while (0)

void log_flush_impl();
#ifdef __cplusplus
} // extern "C"
#endif

#if defined(LOG_FMT)
#ifndef __cplusplus
#error fmt formatting is only available for C++, not C
#endif

#include <fmt/format.h>
#include <fmt/ostream.h>

#define log_format_impl(_fmt, lvl, cat, ...)                                   \
  do {                                                                         \
    std::string str = fmt::format(_fmt __VA_OPT__(, ) __VA_ARGS__);            \
    log_str_impl(str.c_str(), lvl, cat);                                       \
  } while (0)

#define LOG_PATTERN_STR "{}"

#elif defined(LOG_PRINTF)

#define log_format_impl(fmt, lvl, cat, ...)                                    \
  do {                                                                         \
    char buf[1024];                                                            \
    snprintf(buf, sizeof(buf), fmt __VA_OPT__(, ) __VA_ARGS__);                \
    log_str_impl(buf, lvl, cat);                                               \
  } while (0)

#define LOG_PATTERN_STR "%s"

#else
#error You need to specify either LOG_FMT or LOG_PRINTF
#endif
