#pragma once
#include <dirent.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define IPC_NO_DEBUG_DATA

enum shmem_error {
  SHMEM_SHM_FAILED = 1,
  SHMEM_TRUNCATE_FAILED,
  SHMEM_MAP_FAILED,
};

static inline int shmem_create(void **addr, const char *id, size_t size) {
  // memfd_create
  int fd = shm_open(id, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
  if (fd == -1)
    return SHMEM_SHM_FAILED;

  int rc = ftruncate(fd, static_cast<long>(size));
  if (rc != 0)
    return SHMEM_TRUNCATE_FAILED;

  *addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (*addr == MAP_FAILED)
    return SHMEM_MAP_FAILED;

  return 0;
}

static inline int shmem_unmap(void *addr, size_t size) {
  return munmap(addr, size);
}

static inline int shmem_unlink(const char *id) {
  // shm_open cleanup
  return shm_unlink(id);
}

static inline int shmem_unlink_all(const char *prefix) {
  DIR *dir;
  struct dirent *ent;
  dir = opendir("/dev/shm");
  if (dir == NULL)
    return -1;

  while ((ent = readdir(dir)) != NULL) {
    const char *dir_name = ent->d_name;
    int match = 1;

    for (int i = 0; i < (int)strlen(prefix); ++i)
      if (dir_name[i] != prefix[i])
        match = 0;

    if (match)
      shmem_unlink(dir_name);
  }

  closedir(dir);

  return 0;
}

#ifdef __cplusplus
#include "smp.h"
#include <assert.h>
#include <concepts>
#include <tyndall/meta/macro.h>
#include <tyndall/meta/strval.h>
#include <tyndall/meta/typeinfo.h>
#include <tyndall/reflect/reflect.h>
#include <type_traits>
#include <typeindex>

enum shmem_permission {
  SHMEM_READ = 1 << 0,
  SHMEM_WRITE = 1 << 1,
};

/**
 * @brief Concept for shared memory data structures
 *
 * Restricts DATA_STRUCTURE to have a storage type, state and the following
 * methods:
 * - void write(typename DATA_STRUCTURE::storage, typename
 * DATA_STRUCTURE::state)
 * - int read(typename DATA_STRUCTURE::storage, typename DATA_STRUCTURE::state,
 * bool)
 *
 * seq_lock is a shared memory data structure that follows this concept.
 *
 * @tparam DATA_STRUCTURE
 */
template <typename DATA_STRUCTURE>
concept shmem_data_structure = requires(
    DATA_STRUCTURE ds, typename DATA_STRUCTURE::storage storage,
    typename DATA_STRUCTURE::state state) {
  {
    ds.write(storage, state)
  }
  -> std::same_as<void>; // void return type since we don't expect fail on send
  {
    ds.read(storage, state, true)
  } -> std::same_as<int>; // return value: 0 is success, -1 is error, errno is
                          // ENOMSG, EAGAIN

  { typename DATA_STRUCTURE::state{} };
};

/**
 * @brief Shared memory buffer for IPC
 *
 * @tparam DATA_STRUCTURE The data structure to store in the shared memory
 * buffer (e.g. seq_lock). Must fulfill the shmem_data_structure concept.
 * @tparam PERMISSIONS The permissions for the shared memory buffer
 * @tparam ID The id for the shared memory buffer
 */
template <typename DATA_STRUCTURE, int PERMISSIONS, typename ID = strval_t("")>
  requires shmem_data_structure<DATA_STRUCTURE> && (ID::is_strval())
class shmem_buf {
  /// @brief Shared memory buffer for IPC
  void *buf;
  /// @brief State for the data structure
  typename DATA_STRUCTURE::state state;
  /// @brief Type stored in the shared memory buffer
  using storage = typename DATA_STRUCTURE::storage;

  size_t tailer_size;

public:
  shmem_buf() noexcept : buf(NULL) {
    static_assert(ID::occurrences('/') == 0, "Id can't have slashes");

    if constexpr (ID{} != ""_strval)
      init(ID::c_str());
  }

  shmem_buf(const char *id) noexcept {
    static_assert(ID::length() == 0,
                  "Static id should be empty when specifying runtime id");

    init(id);
  }

  /**
   * @brief Convenience function to access the data structure from the shared
   * memory buffer
   *
   * @return DATA_STRUCTURE& The data structure (e.g. seq_lock)
   */
  DATA_STRUCTURE &data_structure() noexcept {
    return *static_cast<DATA_STRUCTURE *>(buf);
  }

  void init(const char *id) noexcept {
    constexpr auto type_hash = typeinfo_hash(DATA_STRUCTURE);

    // #if !defined(__GNUC__) || defined(__clang__)
    // #define IPC_NO_DEBUG_DATA
    // #endif

#ifndef IPC_NO_DEBUG_DATA
    constexpr auto debug_format =
        reflect<typename DATA_STRUCTURE::storage>().get_format();
#endif

    struct {
      std::atomic<std::remove_cv_t<decltype(type_hash)>> th;
#ifndef IPC_NO_DEBUG_DATA
      volatile std::remove_cv_t<decltype(debug_format)> df;
#endif
    } tailer;

    tailer_size = sizeof(tailer);

    constexpr size_t size = sizeof(DATA_STRUCTURE) + sizeof(tailer);
    int rc = shmem_create(&buf, id, size);
    assert(rc == 0);

    // shared memory should be page aligned:
    assert(getpagesize() % CACHELINE_BYTES == 0);
    assert(reinterpret_cast<uintptr_t>(buf) % CACHELINE_BYTES == 0);

    // put type tailer in the end for validation
    // static_assert(
    //     sizeof(tailer) < CACHELINE_BYTES,
    //     "tailer needs to fit in a single section, as aligned by "
    //     M_STRINGIFY(
    //         CACHELINE_BYTES));
    auto tailer_loc =
        reinterpret_cast<decltype(tailer) *>(&data_structure() + 1);

#ifndef IPC_NO_DEBUG_DATA
    tailer_loc->df.set(); // store debug format string
#endif

    if (smp_cmp_xch(tailer_loc->th, 0u, type_hash) == -1)
      assert(tailer_loc->th == type_hash);
  }

  void uninit() noexcept {
    if ((buf != NULL) && (buf != (void *)-1))
      shmem_unmap(buf, sizeof(DATA_STRUCTURE) + tailer_size);
  }

  ~shmem_buf() noexcept { uninit(); }

  /**
   * @brief Write the data to the shared memory buffer
   *
   * @param entry The storage object to write
   */
  void write(const storage &entry) noexcept {
    static_assert(PERMISSIONS & SHMEM_WRITE, "needs write permission");
    data_structure().write(entry, state);
  }

  /**
   * @brief Read the stored data from the shared memory buffer
   *
   * @param entry The storage object to read into
   * @param always_update_entry  If true, the entry will be updated even if
   * returning the same value as a previous call. If false, the output entry
   * will only be updated if the stored entry is new.
   * @return int 0 on success, -1 on error, errno is ENOMSG, EAGAIN
   */
  int read(storage &entry, bool always_update_entry = true) noexcept {
    static_assert(PERMISSIONS & SHMEM_READ, "needs read permission");
    return data_structure().read(entry, state, always_update_entry);
  }

  // disable copy
  shmem_buf(shmem_buf &other) noexcept = delete;

  // enable move
  shmem_buf &operator=(shmem_buf &&other) noexcept {
    uninit();

    buf = other.buf;
    state = other.state;
    other.buf = NULL;

    return *this;
  }

  shmem_buf(shmem_buf &&other) : buf(other.buf), state(other.state) {
    other.buf = NULL; // invalidate shared memory
  }
};

#endif
