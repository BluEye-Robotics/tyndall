#include <cstdio>
#include <tyndall/meta/strval.h>
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

  return 0;
}
