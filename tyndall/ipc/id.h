#pragma once
#include <cassert>
#include <cstring>
#include <string>
#include <tyndall/meta/strval.h>

#ifndef IPC_SHMEM_PREFIX
#define IPC_SHMEM_PREFIX "tyn"
#endif

// Shared memory objects share one flat namespace (/dev/shm on Linux), so a
// topic, which looks like an absolute path, is turned into a file name: the
// prefix IPC_SHMEM_PREFIX and '_', then the topic without its leading slashes
// and with each remaining '/' replaced by '%':
//
//   "/my-process/my_topic" -> "tyn_my-process%my_topic"
//
// "my-topic", "/my-topic" and "//my-topic" are the same topic. A topic can't
// contain '%' (ROS names can't either), so different topics get different
// names. rust-ipc in libblunux builds the same names: change both together.

// Compile time id generation:
// Meant to take strval as template parameter.
template <typename STRING>
using id_remove_leading_slashes =
    decltype(STRING::template remove_leading<'/'>());

template <typename STRING>
using id_replace_slashes = decltype(STRING::template replace<'/', '%'>());

template <typename ID> struct id_prepare_impl {
  static_assert(ID::occurrences('%') == 0,
                "'%' stands for '/' in shared memory names, so an id can't "
                "contain it");
  using type = decltype(create_strval(IPC_SHMEM_PREFIX) + "_"_strval +
                        id_replace_slashes<id_remove_leading_slashes<ID>>{});
};

template <typename ID> using id_prepare = typename id_prepare_impl<ID>::type;

// runtime id generation
template <typename STORAGE>
static inline std::string id_rtid_prepare(const char *id) {
  // remove leading slashes
  while (*id == '/')
    ++id;

  // '%' stands for '/' in shared memory names, so an id can't contain it
  assert(strchr(id, '%') == nullptr);

  std::string prepared_id = IPC_SHMEM_PREFIX "_" + std::string{id};

  for (char &c : prepared_id)
    if (c == '/')
      c = '%';

  return prepared_id;
}
