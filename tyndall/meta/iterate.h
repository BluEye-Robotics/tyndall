#pragma once
#include <type_traits>

/*
  iterate calls function with integral constants from 0 to specified index.
  Useful for iterating recursively at compile time.
*/

template <int index, typename Function>
  requires(index >= 1)
constexpr void iterate_impl(Function f) noexcept {
  constexpr int prev_index = index - 1;

  iterate<prev_index>(f);

  f(std::integral_constant<int, prev_index>());
}

template <int index, typename Function>
  requires(index == 0)
constexpr void iterate_impl(Function f) noexcept {}

template <typename Function>
concept iterable = requires(Function function) {
  function(std::integral_constant<int, 0>{});
  function(std::integral_constant<int, 1>{});
  // we dont bother checking the rest of the integral constants
};

template <int index, typename Function>
  requires(iterable<Function>)
constexpr void iterate(Function f) noexcept {
  return iterate_impl<index>(f);
}
