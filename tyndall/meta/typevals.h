#pragma once
#include <type_traits>

/*
  typevals is a container with entries that have different types
*/

template <typename... Types>
class typevals {};

// Helper: get type at index in Types pack
template <int Index, typename... Types>
struct type_at;

template <typename Head, typename... Tail>
struct type_at<0, Head, Tail...> {
  using type = Head;
};

template <int Index, typename Head, typename... Tail>
struct type_at<Index, Head, Tail...> {
  static_assert(Index < sizeof...(Tail) + 1, "Index out of bounds");
  using type = typename type_at<Index - 1, Tail...>::type;
};

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
  constexpr decltype(auto) get() const noexcept {
    if constexpr (index == sizeof...(Tail)) {
      return val;
    } else {
      return typevals<Tail...>::template get<index>();
    }
  }

  template <int index>
  constexpr decltype(auto) operator[](std::integral_constant<int, index>) const noexcept {
    return get<index>();
  }

  static constexpr int size() noexcept { return 1 + sizeof...(Tail); }

protected:
  template <int index>
  static constexpr typename type_at<index, Type, Tail...>::type get_type() noexcept {
    if constexpr (index == sizeof...(Tail)) {
      return Type();
    } else {
      return typevals<Tail...>::template get_type<index>();
    }
  }
};

template <>
struct typevals<> {
  explicit constexpr typevals() noexcept {}

  template <typename Entry>
  constexpr typevals<Entry> operator+(Entry entry) const noexcept {
    return typevals<Entry>(*this, entry);
  }

  static constexpr int size() noexcept { return 0; }

  template <int index>
  constexpr decltype(auto) get() const noexcept {
    static_assert(index < 0, "Index out of bounds in typevals");
    // This will always trigger a compile error if called
  }

protected:
  template <int index>
  static constexpr int get_type() noexcept {
    static_assert(index < 0, "Index out of bounds in typevals");
    return 0;
  }
};
