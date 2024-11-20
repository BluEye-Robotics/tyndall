#include <tyndall/meta/macro.h>

int main() {
#define RANGE_TEST(i, _) static_assert((0 <= i) && (i < 8));

  M_EVAL(M_RANGE(RANGE_TEST, 0, 8))
}
