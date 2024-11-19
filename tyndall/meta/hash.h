#pragma once

#include <stddef.h>
#include <stdint.h>

// FNV-1a 32bit hashing algorithm.
constexpr uint32_t hash_fnv1a_32(const char *s, size_t count) noexcept {
  return count ? (((hash_fnv1a_32(s, count - 1)) ^
                   static_cast<uint32_t>(s[count - 1])) *
                  16777619u)
               : 2166136261u;
}

constexpr uint32_t operator""_hash_fnv1a_32(const char *s,
                                            size_t size) noexcept {
  return hash_fnv1a_32(s, size);
}

template <typename STRING> constexpr uint32_t hash_fnv1a_32(STRING s) noexcept {
  return hash_fnv1a_32(s.c_str(), s.length());
}

template <typename STRING> constexpr uint32_t hash_fnv1a_32() noexcept {
  return hash_fnv1a_32(STRING::c_str(), STRING::length());
}
