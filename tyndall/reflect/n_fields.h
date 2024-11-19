#pragma once
#include "tyndall/meta/typeinfo.h"
#include <climits>
#include <cstdio>
#include <type_traits>
#include <typeinfo>
#include <utility>

// based on
// https://github.com/apolukhin/pfr_non_boost/blob/91f801bb08cf817c640d9c6d57ddfad5f38de92d/include/pfr/detail/fields_count.hpp
// , with license:
// https://github.com/apolukhin/pfr_non_boost/blob/91f801bb08cf817c640d9c6d57ddfad5f38de92d/LICENSE_1_0.txt

namespace n_fields_detail {
template <size_t... I>
using index_sequence = std::integer_sequence<size_t, I...>;

template <size_t N>
using make_index_sequence = std::make_integer_sequence<size_t, N>;

template <typename T> constexpr T unsafe_declval() noexcept {
  constexpr typename std::remove_reference_t<T> *ptr = 0;
  return static_cast<T>(*ptr);
}

struct ubiq_lref_constructor {
  int ignore;
  template <typename T> constexpr operator T &() const && noexcept {
    return unsafe_declval<T &>();
  }

  template <typename T> constexpr operator T &() const & noexcept {
    return unsafe_declval<T &>();
  }
};

struct ubiq_rref_constructor {
  int ignore;
  template <typename T> constexpr operator T() const && noexcept {
    return unsafe_declval<T>();
  }
};

template <typename T, size_t... I>
  requires(std::is_copy_constructible_v<T>)
constexpr auto constructible(index_sequence<I...>) noexcept ->
    typename std::add_pointer_t<decltype(T{ubiq_lref_constructor{I}...})>;

template <typename T, size_t... I>
  requires(!std::is_copy_constructible_v<T>)
constexpr auto constructible(index_sequence<I...>) noexcept ->
    typename std::add_pointer_t<decltype(T{ubiq_rref_constructor{I}...})>;

template <class T, size_t N,
          typename = decltype(constructible<T>(make_index_sequence<N>()))>
using constructible_t = size_t;

template <typename T, size_t N>
constexpr auto n_fields_linear(long) noexcept -> constructible_t<T, N> {
  return N;
}

template <typename T, size_t N> constexpr size_t n_fields_linear(int) noexcept {
  return n_fields_linear<T, N - 1>(0l);
}

template <typename T, size_t N>
constexpr auto n_fields_binary(long) noexcept -> constructible_t<T, N> {
  return N;
}

template <typename T, size_t N> constexpr size_t n_fields_binary(int) noexcept {
  return n_fields_binary<T, N / 2>(0l);
}
} // namespace n_fields_detail

template <typename T> constexpr size_t n_fields() noexcept {
  constexpr size_t max_field_count = sizeof(T) * CHAR_BIT;

  using type = std::remove_cvref_t<T>;

  static_assert(std::is_aggregate<type>::value || std::is_scalar<type>::value,
                "T must be aggregate initializable");

  // Binary search is applied to avoid too large template depth.
  constexpr size_t binary_search =
      n_fields_detail::n_fields_binary<type, max_field_count>(0l);

  return n_fields_detail::n_fields_linear<type, binary_search * 2 + 1>(0l);
}
