#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <tyndall/meta/strval.h>

// print_format_typeid returns the typeid character for basic types and empty
// otherwise
template <typename T> constexpr auto print_format_typeid() noexcept {
  return ""_strval;
}

template <> constexpr auto print_format_typeid<bool>() noexcept {
  return "b"_strval;
}

template <> constexpr auto print_format_typeid<float>() noexcept {
  return "f"_strval;
}

template <> constexpr auto print_format_typeid<double>() noexcept {
  return "d"_strval;
}

template <> constexpr auto print_format_typeid<int>() noexcept {
  return "i"_strval;
}

template <> constexpr auto print_format_typeid<unsigned int>() noexcept {
  return "j"_strval;
}

template <> constexpr auto print_format_typeid<short>() noexcept {
  return "s"_strval;
}

template <> constexpr auto print_format_typeid<unsigned short>() noexcept {
  return "t"_strval;
}

template <> constexpr auto print_format_typeid<char>() noexcept {
  return "c"_strval;
}

template <> constexpr auto print_format_typeid<unsigned char>() noexcept {
  return "h"_strval;
}

template <> constexpr auto print_format_typeid<long>() noexcept {
  return "l"_strval;
}

template <> constexpr auto print_format_typeid<unsigned long>() noexcept {
  return "m"_strval;
}

template <> constexpr auto print_format_typeid<long long>() noexcept {
  return "x"_strval;
}

template <> constexpr auto print_format_typeid<unsigned long long>() noexcept {
  return "y"_strval;
}

// TODO:
// template<>
// constexpr auto print_format_get<const char*>() noexcept
//{
//  return "s"_strval;
//}

// print_format_printf returns the printf format for basic types and empty
// otherwise
template <typename T> constexpr auto print_format_printf() noexcept {
  return ""_strval;
}

template <> constexpr auto print_format_printf<bool>() noexcept {
  return "%d"_strval;
}

template <> constexpr auto print_format_printf<float>() noexcept {
  return "%f"_strval;
}

template <> constexpr auto print_format_printf<double>() noexcept {
  return "%f"_strval;
}

template <> constexpr auto print_format_printf<int>() noexcept {
  return "%d"_strval;
}

template <> constexpr auto print_format_printf<unsigned int>() noexcept {
  return "%u"_strval;
}

template <> constexpr auto print_format_printf<short>() noexcept {
  return "%d"_strval;
}

template <> constexpr auto print_format_printf<unsigned short>() noexcept {
  return "%u"_strval;
}

template <> constexpr auto print_format_printf<char>() noexcept {
  return "%c"_strval;
}

template <> constexpr auto print_format_printf<unsigned char>() noexcept {
  return "%u"_strval;
}

template <> constexpr auto print_format_printf<long>() noexcept {
  return "%ld"_strval;
}

template <> constexpr auto print_format_printf<unsigned long>() noexcept {
  return "%u"_strval;
}

template <> constexpr auto print_format_printf<long long>() noexcept {
  return "%lld"_strval;
}

template <> constexpr auto print_format_printf<unsigned long long>() noexcept {
  return "%llu"_strval;
}

template <typename T> size_t print_format_alignment(const char *ptr) {
  size_t alignment_error = reinterpret_cast<const uintptr_t>(ptr) % sizeof(T);
  return (alignment_error > 0) ? sizeof(T) - alignment_error : 0;
}

template <typename Fmt> size_t print_format(const char *data) {
  const size_t align = print_format_alignment<Fmt>(data);
  printf((print_format_printf<Fmt>() + print_format_typeid<Fmt>()).c_str(),
         *(Fmt *)(data + align));
  return sizeof(Fmt) + align;
}

static inline size_t print_format(char c, const char *data) {
  switch (c) {
  case print_format_typeid<bool>().get<0>():
    return print_format<bool>(data);
  case print_format_typeid<float>().get<0>():
    return print_format<float>(data);
  case print_format_typeid<double>().get<0>():
    return print_format<double>(data);
  case print_format_typeid<int>().get<0>():
    return print_format<int>(data);
  case print_format_typeid<unsigned int>().get<0>():
    return print_format<unsigned int>(data);
  case print_format_typeid<short>().get<0>():
    return print_format<short>(data);
  case print_format_typeid<unsigned short>().get<0>():
    return print_format<unsigned short>(data);
  case print_format_typeid<char>().get<0>():
    return print_format<char>(data);
  case print_format_typeid<unsigned char>().get<0>():
    return print_format<unsigned char>(data);
  case print_format_typeid<long>().get<0>():
    return print_format<long>(data);
  case print_format_typeid<unsigned long>().get<0>():
    return print_format<unsigned long>(data);
  case print_format_typeid<long long>().get<0>():
    return print_format<long long>(data);
  case print_format_typeid<unsigned long long>().get<0>():
    return print_format<unsigned long long>(data);
  default:
    return 0;
  }
}
