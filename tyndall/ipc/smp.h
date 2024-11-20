#pragma once
#ifdef __cplusplus
#include <atomic>
#include <cassert>
#include <type_traits>
#else
#include <stdatomic.h>
#endif

// Symmetric multiprocessing helpers, using a mix of inline asm from the Linux
// kernel, and C11 atomics

#define barrier() __asm__ __volatile__("" : : : "memory")

#if __arm__ || __aarch64__ // tuned for armv7 (32-bit)

#define CACHELINE_BYTES 32
// L1:
// https://developer.arm.com/documentation/ddi0388/f/Level-1-Memory-System/About-the-L1-memory-system
// L2: https://community.nxp.com/thread/510105

// Armv7 memory barriers, as per:
// https://github.com/torvalds/linux/blob/master/arch/arm/include/asm/barrier.h
// https://developer.arm.com/documentation/dui0489/c/arm-and-thumb-instructions/miscellaneous-instructions/dmb--dsb--and-isb
// https://developer.arm.com/documentation/genc007826/latest
// http://www.cl.cam.ac.uk/~pes20/ppc-supplemental/test7.pdf
#define smp_dmb(option) __asm__ __volatile__("dmb " #option : : : "memory")
#define smp_mb() smp_dmb(ish)
#define smp_wmb() smp_dmb(ishst)
#ifdef __aarch64__
#define smp_rmb() smp_dmb(ishld); // armv8
#else
#define smp_rmb() smp_mb()
#endif

// C/C++11
// #define smp_mb() std::atomic_signal_fence(std::memory_order_acq_rel) // c11
// implementation
////#define smp_rmb() smp_mb()
////#define smp_wmb() smp_mb()
// #define smp_rmb() std::atomic_signal_fence(std::memory_order_acquire)
// #define smp_wmb() std::atomic_signal_fence(std::memory_order_release)

// https://patchwork.kernel.org/patch/1361581/
#define cpu_relax()                                                            \
  do {                                                                         \
    asm("nop");                                                                \
    asm("nop");                                                                \
    asm("nop");                                                                \
    asm("nop");                                                                \
    asm("nop");                                                                \
    smp_dmb(ish);                                                              \
  } while (0)

#elif __x86_64__
#define CACHELINE_BYTES 64

#define smp_mb() barrier()
#define smp_wmb() barrier()
#define smp_rmb() barrier()

#define cpu_relax() barrier()
#endif

// Protected variable accesses as per
// http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2018/p0124r5.html#Variable%20Access

// KERNEL
#define smp_read_once(x) (*(const volatile __typeof__(x) *)&(x))
#define smp_write_once(x, val)                                                 \
  do {                                                                         \
    *(volatile __typeof__(x) *)&(x) = (val);                                   \
  } while (0)

// C11
// #ifdef __cplusplus
// #define smp_read_once(x)
// (std::atomic_load_explicit((std::atomic<__typeof__(x)>*)&(x),
// std::memory_order_acquire)) #define smp_write_once(x, val)
// std::atomic_store_explicit((std::atomic<__typeof__(x)>*)&(x), val,
// std::memory_order_release) #else #define smp_read_once(x)
// ((atomic_load_explicit((&(x)), memory_order_acquire))) #define
// smp_write_once(x, val) atomic_store_explicit((&(x)), val,
// memory_order_release) #endif

// compare exchange
#ifdef __cplusplus
template <typename T>
int smp_cmp_xch(std::atomic<T> &x, T expected, T desired) {
  assert(x.is_lock_free());
  return std::atomic_compare_exchange_strong(&x, &expected, desired) ? 0 : -1;
}
#else
#define smp_cmp_xch(x, expected, desired)                                      \
  ({                                                                           \
    static_assert(false, "no smp_cmp_xch for c implemented\n");                \
    0;                                                                         \
  }) // TODO
#endif
