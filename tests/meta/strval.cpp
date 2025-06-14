#include <cassert>
#include <cstring>
#include <tyndall/meta/strval.h>
#include <tyndall/meta/typeinfo.h>
#include <type_traits>


int main() {
  constexpr auto hei = "hei"_strval;
  decltype(hei){}.c_str();
  static_assert(hei.length() == 3);
  static_assert(hei + "du"_strval == "heidu"_strval);
  static_assert(("hei din sei"_strval).occurrences('i') == 3);
  static_assert(("a_b_c"_strval).replace<'_', '-'>() == "a-b-c"_strval);
  static_assert(("///hei"_strval).remove_leading<'/'>() == hei);
  static_assert(to_strval<42>{} == "42"_strval);
  static_assert(sizeof(hei) == 4);
  static_assert(sizeof(""_strval) == 1);

  {
    char buf[100];
    memset(buf, 0, sizeof(buf));
    void *buf_p = buf;
    auto hei_p = static_cast<std::remove_cvref_t<decltype(hei)> *>(buf_p);
    *hei_p = {};
    assert(strcmp(buf, hei.c_str()) == 0);
  }

  {
    assert("abcdef"_strval.length() == 6);
    assert("abcdef"_strval.occurrences('a') == 1);
    assert("abcdef"_strval.occurrences('b') == 1);
    assert("abcdef"_strval.occurrences('c') == 1);
    assert("abcdef"_strval.occurrences('d') == 1);
    assert("abcdef"_strval.occurrences('e') == 1);
    assert("abcdef"_strval.occurrences('f') == 1);
    assert("abcdef"_strval.occurrences('g') == 0);
  }

  {
    assert("/test/standard"_strval.length() == 14);
    assert("/test/standard"_strval.occurrences('/') == 2);
    assert("/test/standard"_strval.occurrences('t') == 3);
    assert("/test/standard"_strval.occurrences('e') == 1);
    assert("/test/standard"_strval.occurrences('s') == 2);
    assert("/test/standard"_strval.occurrences('a') == 2);

  }
}
