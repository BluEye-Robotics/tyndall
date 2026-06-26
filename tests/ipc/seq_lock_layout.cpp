// Layout regression test for issue #16.
//
// seq_lock<STORAGE> must place `entry` on its own cacheline so the sequence
// counter (seq/size) and the payload do not false-share. The offset of `entry`
// must be exactly CACHELINE_BYTES on every ABI, independent of STORAGE's
// alignment and of the padding the compiler inserts between `seq` (4B) and
// `size` (8B on 64-bit targets).
//
// Before the fix, on 64-bit targets `entry` landed at 36/40 (aarch64) or 68/72
// (x86_64) instead of CACHELINE_BYTES, so these static_asserts would fail to
// compile.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <tyndall/ipc/seq_lock.h>
#include <tyndall/meta/macro.h>

// Storage types spanning both alignment classes.
using s_char = char;        // 1-byte aligned
using s_float = float;      // 4-byte aligned
struct s_vec3 {             // 8-byte aligned (the Vector3 repro)
  double x, y, z;
};
struct s_mixed {            // mixed scalars, 8-byte aligned
  long a;
  double b;
  char c;
  unsigned long d;
};

template <typename STORAGE> constexpr void check_layout() {
  static_assert(offsetof(seq_lock<STORAGE>, entry) == CACHELINE_BYTES,
                "entry must start exactly one cacheline into seq_lock");
  static_assert(offsetof(seq_lock<STORAGE>, entry) % CACHELINE_BYTES == 0,
                "entry must be cacheline-aligned");
}

template void check_layout<s_char>();
template void check_layout<s_float>();
template void check_layout<s_vec3>();
template void check_layout<s_mixed>();

#define check(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf(__FILE__ ":" M_STRINGIFY(__LINE__) " Assertion failed: " #cond    \
                                                "\n");                         \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)

template <typename STORAGE> void check_runtime(const char *name) {
  seq_lock<STORAGE> s{};
  const size_t off =
      reinterpret_cast<char *>(&s.entry) - reinterpret_cast<char *>(&s);
  printf("seq_lock<%s>: offsetof(entry)=%zu CACHELINE_BYTES=%d sizeof=%zu\n",
         name, off, (int)CACHELINE_BYTES, sizeof(seq_lock<STORAGE>));
  check(off == CACHELINE_BYTES);
}

int main() {
  check_runtime<s_char>("char");
  check_runtime<s_float>("float");
  check_runtime<s_vec3>("vec3");
  check_runtime<s_mixed>("mixed");
  return 0;
}
