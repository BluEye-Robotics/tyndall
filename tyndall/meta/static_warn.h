#pragma once
#include <tyndall/meta/macro.h>
#include <type_traits>

namespace static_warn_detail {
template <int test> struct converter : public std::true_type {};
template <> struct converter<0> : public std::false_type {};
} // namespace static_warn_detail

#define static_warn(cond, msg)                                                 \
  struct M_CAT(static_warning, __LINE__) {                                     \
    void _(std::false_type const &) __attribute__((deprecated(msg))) {};       \
    void _(std::true_type const &) {};                                         \
    M_CAT(static_warning, __LINE__)() {                                        \
      _(static_warn_detail::converter<(cond)>());                              \
    }                                                                          \
  }
