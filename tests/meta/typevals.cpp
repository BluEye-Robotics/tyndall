#include <tyndall/meta/iterate.h>
#include <tyndall/meta/typevals.h>

int main() {
  {
    constexpr struct A {
      int a;
      float b;
      bool operator==(const A &) const = default;
    } a{4, 2};

    constexpr struct B {
      char a;
      double b;
      long c;
      bool operator==(const B &) const = default;
    } b{6, 6, 6};

    static constexpr auto tv = typevals{} + 5 + b + a + B{0, 0, 7};

    static_assert(tv.size() == 4);
    static_assert(tv.get<0>() == 5);
    static_assert(tv.get<1>() == b);
    static_assert(tv.get<2>() == a);
    static_assert(tv.get<3>() == B{0, 0, 7});

    static_assert(tv[std::integral_constant<int, 2>()] == a);

    iterate<tv.size()>([a, b](auto index) {
      static_assert((index >= 0) && (index < tv.size()));

      if constexpr (index == 0)
        static_assert(tv[index] == 5);
      else if constexpr (index == 1)
        static_assert(tv[index] == b);
      else if constexpr (index == 2)
        static_assert(tv[index] == a);
      else if constexpr (index == 3)
        static_assert(tv[index] == B{0, 0, 7});
    });
  }
}
