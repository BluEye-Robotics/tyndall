#include <array>
#include <tyndall/reflect/n_fields.h>

#include <string>

int main() {
  {
    struct S {
      float floating_point = 3.14f;
      int integer = 42;
      unsigned char unsigned_char = 255;
      std::string ss;
      std::array<int, 10> a;
    };

    S s;

    static_assert(n_fields<decltype(s)>() == 5);
  }
}
