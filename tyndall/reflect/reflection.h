#pragma once
#include <type_traits>
#include <utility>

#include "print_format.h"
#include "tyndall/meta/strval.h"
#include "tyndall/meta/typeinfo.h"

template <typename T> constexpr auto reflect_aggregate() noexcept;

template <typename... Args> struct reflection {};

template <typename Lhs, typename... Rhs>
struct reflection<Lhs, Rhs...> : public reflection<Rhs...> {
  const Lhs &lhs;

  explicit constexpr reflection(Lhs &&lhs, Rhs &&...rhs) noexcept
      : reflection<Rhs...>{std::forward<Rhs>(rhs)...}, lhs(lhs) {}

  template <size_t index> constexpr const auto &get() const noexcept {
    return get_impl<index>(*this);
  }

  static constexpr size_t size() noexcept { return 1 + sizeof...(Rhs); }

  static constexpr auto get_format() noexcept {
    constexpr auto lhs_format =
        ::print_format_typeid<std::remove_cvref_t<Lhs>>();

    constexpr auto rhs_format = reflection<Rhs...>::get_format();

    if constexpr (lhs_format == ""_strval)
      return reflect_aggregate<Lhs>().get_format() + rhs_format;
    else
      return lhs_format + rhs_format;
  }

protected:
  template <size_t index>
    requires(index == 0)
  static constexpr const Lhs &
  get_impl(const reflection<Lhs, Rhs...> &refl) noexcept {
    return refl.lhs;
  }

  template <size_t index>
    requires(index > 0)
  static constexpr const auto &
  get_impl(const reflection<Lhs, Rhs...> &refl) noexcept {
    return reflection<Rhs...>::template get_impl<index - 1>(
        static_cast<const reflection<Rhs...> &>(refl));
  }
};

template <> struct reflection<> {
public:
  static constexpr size_t size() noexcept { return 0; }

  static constexpr auto get_format() noexcept { return ""_strval; }
};

#include "reflect.h"
