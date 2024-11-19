#pragma once
#include <type_traits>

/*
  typevals is a container with entries that have different types
*/

template <typename... Types> class typevals {};

template <typename Type, typename... Tail>
struct typevals<Type, Tail...> : public typevals<Tail...> {
  Type val;

  explicit constexpr typevals(typevals<Tail...> tl, Type val) noexcept
      : typevals<Tail...>(tl), val(val) {}

  template <typename Entry>
  constexpr typevals<Entry, Type, Tail...>
  operator+(Entry entry) const noexcept {
    return typevals<Entry, Type, Tail...>(*this, entry);
  }

  template <int index>
    requires(index == sizeof...(Tail))
  constexpr const Type &get() const noexcept {
    return val;
  }

  template <int index>
    requires(index < sizeof...(Tail))
  constexpr decltype(typevals<Tail...>::template get_type<index>())
  get() const noexcept {
    return typevals<Tail...>::template get<index>();
  }

  template <int index>
  constexpr decltype(typevals<Type, Tail...>::template get_type<index>())
  operator[](std::integral_constant<int, index>) const noexcept {
    return get<index>();
  }

  static constexpr int size() noexcept { return 1 + sizeof...(Tail); }

protected:
  // get_type is a static helper for determining return type of get
  template <int index>
    requires(index == sizeof...(Tail))
  static constexpr const Type &get_type() noexcept {
    return Type();
  }

  template <int index>
    requires(index < sizeof...(Tail))
  static constexpr decltype(typevals<Tail...>::template get_type<index>())
  get_type() noexcept {
    return typevals<Tail...>::template get_type<index>();
  }
};

template <> struct typevals<> {
  explicit constexpr typevals() noexcept {}

  template <typename Entry>
  constexpr typevals<Entry> operator+(Entry entry) const noexcept {
    return typevals<Entry>(*this, entry);
  }

  static constexpr int size() noexcept { return 0; }

protected:
  template <int index> static constexpr int get_type() noexcept { return 0; }
};
