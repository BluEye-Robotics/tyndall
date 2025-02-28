#pragma once
#include "n_fields.h"
#include "reflection.h"
#include "tyndall/meta/typeinfo.h"
#include <cstddef>
#include <tyndall/meta/macro.h>
#include <tyndall/meta/static_warn.h>
#include <type_traits>

// based on
// https://github.com/apolukhin/pfr_non_boost/blob/91f801bb08cf817c640d9c6d57ddfad5f38de92d/include/pfr/detail/core17_generated.hpp
// , with license:
// https://github.com/apolukhin/pfr_non_boost/blob/91f801bb08cf817c640d9c6d57ddfad5f38de92d/LICENSE_1_0.txt

#ifndef REFLECT_MAX_FIELDS
#define REFLECT_MAX_FIELDS 100
#endif
static_assert(M_INC(REFLECT_MAX_FIELDS) < M_MAX_ARITHMETIC,
              "REFLECT_MAX_FIELDS must be less than M_MAX_ARITHMETIC, "
              "specified in <tyndall/meta/macro.h>");

template <size_t I> using size_t_ = std::integral_constant<size_t, I>;

template <typename T> constexpr auto reflect_impl(T &&t, size_t_<0>) noexcept {
  return reflection<>{};
}

#define REFLECT_BINDING(I, _) M_CAT(a, I)
#define REFLECT_DECLTYPE_BINDING(I, _) decltype(M_CAT(a, I))
#define REFLECT_FORWARD_DECLTYPE_BINDING(I, _)                                 \
  std::forward<decltype(M_CAT(a, I))>(M_CAT(a, I))

#define REFLECT_I(I, _)                                                        \
  template <typename T>                                                        \
  constexpr auto reflect_impl(T &&t, size_t_<I>) noexcept {                    \
    auto &&[M_RANGE_WITH_COMMA(REFLECT_BINDING, 0, I)] = t;                    \
    return reflection<M_RANGE_WITH_COMMA(REFLECT_DECLTYPE_BINDING, 0, I)>{     \
        M_RANGE_WITH_COMMA(REFLECT_FORWARD_DECLTYPE_BINDING, 0, I)};           \
  }
M_EVAL(M_RANGE(REFLECT_I, 1, M_INC(REFLECT_MAX_FIELDS)))

// The range above produces items like the following (for I=3):
// template<typename T>
// constexpr auto reflect_impl(T&& t, size_t_<3>) noexcept
//{
//  auto&& [a0, a1, a2] = t;
//  return reflection<decltype(a0), decltype(a1),
//  decltype(a2)>{std::forward<decltype(a0)>(a0),
//  std::forward<decltype(a1)>(a1), std::forward<decltype(a2)>(a2)};
//}

template <typename T, size_t I>
constexpr reflection<> reflect_impl(T &&, size_t_<I>) noexcept {
  static_assert(sizeof(T) && false,
                "number of fields cannot exceed REFLECT_MAX_FIELDS");
  return {};
}

template <typename T>
  requires(!std::is_class_v<std::remove_cvref_t<T>>)
constexpr auto reflect_scalar(T &&t) noexcept {
  static_assert(std::is_scalar_v<std::remove_cvref_t<T>>, "T must be scalar");
  return reflection<decltype(t)>{std::forward<T>(t)};
}

template <typename T>
  requires(std::is_class_v<std::remove_cvref_t<T>>)
constexpr auto reflect_scalar(T &&t) noexcept {
  auto &&[a0] = t;
  return reflection<decltype(a0)>{std::forward<decltype(a0)>(a0)};
}

template <typename T> constexpr auto reflect_aggregate(T &&t) noexcept {
  using type = const std::remove_cvref_t<T>;

  static_assert(!std::is_union_v<type>, "unions are not supported");

  constexpr size_t number_of_fields = n_fields<type>();
  static_assert(number_of_fields <= REFLECT_MAX_FIELDS,
                "Struct too large! Maybe increase REFLECT_MAX_FIELDS");
  return reflect_impl<type>(std::forward<type>(t), size_t_<number_of_fields>{});
}

template <typename T> constexpr auto reflect_aggregate() noexcept {
  static_assert(std::is_aggregate_v<std::remove_cvref_t<T>>,
                "T must be aggregate");

  return reflect_aggregate(T{});
}

template <typename T> constexpr auto reflect(T &&t) noexcept {
  if constexpr (n_fields<T>() == 1)
    return reflect_scalar<T>(std::forward<T>(t));
  else
    return reflect_aggregate(std::forward<T>(t));
}

template <typename T> constexpr auto reflect() noexcept {
  using type = std::remove_cvref_t<T>;

  constexpr bool aggregate_initializable =
      std::is_aggregate_v<type> || std::is_scalar_v<type>;
  static_warn(aggregate_initializable, "T must be aggregate initializable");

  if constexpr (aggregate_initializable)
    return reflect(T{});
  else
    return reflection<>{};
}
