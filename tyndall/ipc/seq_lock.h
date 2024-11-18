/*
seq_lock.h

Sequence lock: https://en.wikipedia.org/wiki/Seqlock,
https://lwn.net/Articles/21355/ Supports single writer, multiple readers Reader
always gets the most recent entry

Inspired by:
http://www.1024cores.net/home/lock-free-algorithms/reader-writer-problem/improved-lock-free-seqlock
https://github.com/torvalds/linux/blob/master/include/vdso/helpers.h
https://github.com/rigtorp/Seqlock
*/

#pragma once
#include "smp.h"
#include <assert.h>
#include <errno.h>

__attribute__((always_inline)) static inline unsigned
seq_lock_write_begin(unsigned *seq) {
  unsigned seq1 = *seq;

  smp_write_once(*seq, ++seq1);
  smp_wmb();

  return seq1;
}

__attribute__((always_inline)) static inline void
seq_lock_write_end(unsigned *seq, unsigned seq1) {
  smp_wmb();
  smp_write_once(*seq, ++seq1);
}

__attribute__((always_inline)) static inline unsigned
seq_lock_read_begin(unsigned *seq) {
  unsigned seq1;
  while ((seq1 = smp_read_once(*seq)) & 1)
    cpu_relax();

  smp_rmb();

  return seq1;
}

__attribute__((always_inline)) static inline unsigned
seq_lock_read_retry(unsigned *seq, unsigned seq1) {
  smp_rmb();
  unsigned seq2 = smp_read_once(*seq);

  return seq2 != seq1;
}

#ifdef __cplusplus

struct seq_lock_state {
  unsigned prev_seq = 0;
  bool has_read_once = false;
};

template <typename STORAGE> struct seq_lock {
  static_assert(std::is_nothrow_copy_assignable_v<STORAGE>);
  static_assert(std::is_trivially_copy_assignable_v<STORAGE>);

  unsigned seq;

  char padding[CACHELINE_BYTES - sizeof(seq)];

  STORAGE entry;

public:
  using storage = STORAGE;
  using state = seq_lock_state;

  void write(const STORAGE &entry, seq_lock_state &) {
    unsigned seq = seq_lock_write_begin(&this->seq);

    this->entry = entry;

    seq_lock_write_end(&this->seq, seq);
  }

  int read(STORAGE &entry, seq_lock_state &state) {
    int rc;
    unsigned seq1;

    do {
      seq1 = seq_lock_read_begin(&this->seq);
      entry = this->entry;

    } while (seq_lock_read_retry(&this->seq, seq1));

    if (seq1 != state.prev_seq) {
      rc = 0;
      state.prev_seq = seq1;
      if (!state.has_read_once)
        state.has_read_once = true;
    } else if ((seq1 == 0) && !state.has_read_once) {
      rc = -1;
      errno = ENOMSG;
    } else {
      rc = -1;
      errno = EAGAIN;
    }
    return rc;
  }
} __attribute__((aligned(CACHELINE_BYTES)));

#endif
