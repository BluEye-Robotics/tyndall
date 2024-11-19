#include <cstdint>

#define LOG_PRINTF
#include "log.h"

void log_flush_impl() { fflush(stdout); }

const char *to_string(log_level_t lvl) {
  switch (lvl) {
  case log_level_error:
    return "error";
  case log_level_warning:
    return "warning";
  case log_level_info:
    return "info";
  case log_level_debug:
    return "debug";
  default:
    return "unknown";
  }
}

const char *to_color_string(log_level_t lvl) {
  switch (lvl) {
  case log_level_error:
    return log_color_red log_color_bold "error" log_color_reset;
  case log_level_warning:
    return log_color_yellow log_color_bold "warning" log_color_reset;
  case log_level_info:
    return log_color_cyan log_color_bold "info" log_color_reset;
  case log_level_debug:
    return log_color_green log_color_bold "debug" log_color_reset;
  default:
    return "unknown";
  }
}

log_level_t log_level(const char *cat) {
  log_level_t ref_lvl = (log_level_t)-1;
  const char *env = getenv("LOG_LEVEL");
  if (env) {
    if ((cat == NULL) || strlen(cat) == 0) {
      for (int i = (int)log_level_debug; i <= (int)log_level_error; ++i)
        if (strncmp(env, to_string((log_level_t)i),
                    strlen(to_string((log_level_t)i))) == 0)
          ref_lvl = (log_level_t)i;

      if ((int)ref_lvl == -1)
        ref_lvl = log_level_info;
    } else {
      for (const char *e = env; (e != NULL) && (e[1] != '\0');
           e = strchr(e + 1, ',')) {
        if (*e == ',')
          ++e;

        const char *colon = strchr(e, ':');
        const char *comma = strchr(e, ',');

        bool matches = true;
        {
          const char *glob_start = e;
          const char *cat_match = cat;
          const char *glob;

          do {
            glob = strchr(glob_start, '*') ?: strchr(glob_start, ':') ?: strchr(glob_start, ',') ?: strchr(glob_start, '\0');

            if (glob_start == e) // first iteration
            {
              if (strncmp(glob_start, cat_match,
                          static_cast<size_t>(glob - glob_start)) != 0)
                matches = false;
            } else {
              bool cat_match_matches = false;
              for (const char *c = cat_match;
                   !cat_match_matches && (*c != '\0'); ++c) {
                if (strncmp(c, glob_start,
                            static_cast<size_t>(glob - glob_start)) == 0) {
                  cat_match_matches = true;
                  cat_match = c;
                }
              }
              if (!cat_match_matches)
                matches = false;
            }

            cat_match += glob - glob_start;
            glob_start = glob + 1;
          } while (*glob == '*');

          // check tail
          if ((glob[-1] != '*') && (cat_match - cat != (int)strlen(cat)))
            matches = false;
        }

        if (matches) {
          if ((colon != NULL) && (colon[1] != '\0') &&
              ((comma == NULL) || (comma > colon))) {
            for (int i = (int)log_level_debug; i <= (int)log_level_error; ++i)
              if (strncmp(colon + 1, to_string((log_level_t)i),
                          strlen(to_string((log_level_t)i))) == 0)
                ref_lvl = (log_level_t)i;
          }
        }
      }

      if ((int)ref_lvl == -1)
        ref_lvl = log_level("");
    }
  } else
    ref_lvl = log_level_info;

  return ref_lvl;
}
