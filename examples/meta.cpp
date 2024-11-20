#include <cstdio>
#include <tyndall/meta/iterate.h>
#include <tyndall/meta/strval.h>
#include <tyndall/meta/typevals.h>
#include <typeinfo>

template <typename T> struct tstruct {
  using Type = T;
  T a;
};

int main() {
  constexpr auto sv = "hei"_strval;
  printf("strval: %s\n", sv.c_str());
  printf("strval from type: %s\n", decltype(sv)::c_str());
  printf("strval length: %d\n", (""_strval).length());
  printf("strval sum: %s\n", ("hei"_strval + "du"_strval).c_str());
  printf("occurrences: %d\n", ("hei din sei"_strval).occurrences('i'));
  printf("replace: %s\n", ("hei din sei"_strval).replace<'i', 'y'>().c_str());
  printf("remove_leading: %s\n",
         ("///hei din sei"_strval).remove_leading<'/'>().c_str());
  printf("to_strval: %s\n", to_strval<42>::c_str());

  {
    printf("typevals:\n");
    {
      struct A {
        int a;
        float b;
      };
      struct B {
        int a;
        float b;
      };
      static constexpr auto col = typevals{} + 5 + A{4, 2} + B{3, 9};
      printf("col size: %d\n", col.size());

      constexpr auto res = col.get<2>();
      printf("col res: %s\n", typeid(res).name());
      constexpr auto res2 = col[std::integral_constant<int, 2>()];
      printf("col res2: %s\n", typeid(res2).name());
    }

    {
      static constexpr auto c = typevals{} + tstruct<int>{5};
      constexpr auto res = c.get<0>();
      printf("c res: %s\n", typeid(res).name());
      decltype(res)::Type t;
      (void)sizeof(t);
    }

    {
      static constexpr auto c = typevals{} + tstruct<int>{-5} +
                                tstruct<float>{3} + tstruct<unsigned>{8};

      iterate<c.size()>([](auto index) {
        printf("iteration %d: %s\n", index(), typeid(index).name());
      });
    }
  }

  return 0;
}
