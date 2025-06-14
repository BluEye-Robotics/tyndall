#include <array>
#include <string>
#include <tyndall/reflect/reflect.h>
#include <typeinfo>

int main() {

  {
    struct S {
      float floating_point;
      int integer;
      unsigned char unsigned_char = 5;
    };

    constexpr S s{
        .floating_point = 3.14f,
        .integer = 42,
        .unsigned_char = 255,
    };

    constexpr auto floating_point = reflect(s).get<0>();
    static_assert(floating_point == s.floating_point);

    constexpr auto integer = reflect(s).get<1>();
    static_assert(integer == s.integer);

    constexpr auto unsigned_char = reflect(s).get<2>();
    static_assert(unsigned_char == s.unsigned_char);

    constexpr size_t size = reflect(s).size();
    static_assert(size == 3);
  }

  {
    struct A {
      int a;
      int b;
    };

    struct S {
      float floating_point;
      int integer;
      int another_integer;
      unsigned int unsigned_integer;
      unsigned char unsigned_char;
      unsigned short signed_short;
      A a;
      std::array<unsigned int, 3> b;
    };

    constexpr S s{};
    constexpr size_t size = reflect(s).size();
    static_assert(size == 8);

    constexpr auto b = reflect(s).get<7>();
    static_assert(std::is_same_v<std::remove_cv_t<decltype(b)>,
                                 std::remove_cv_t<decltype(s.b)>>);

    static_assert(reflect(s).get_format() == "fiijhtiijjj"_strval);
  }

  {
    constexpr float s = 3.14f;
    constexpr size_t size = reflect(s).size();
    static_assert(size == 1);

    constexpr auto floating_point = reflect(s).get<0>();
    static_assert(floating_point == s);
  }

  {
    struct S {
      int a;
      unsigned char b;
      float c, d;
      S() {}
    } msg;
    static_assert(!std::is_aggregate_v<decltype(msg)> &&
                  !std::is_scalar_v<decltype(msg)>);
    static_assert(reflect<decltype(msg)>().get_format() == ""_strval);
    static_assert(sizeof(reflect<decltype(msg)>().get_format()) == 1);
    static_assert(reflect<decltype(msg)>().size() == 0);
  }
}
