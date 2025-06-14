#pragma once
#include <string>
#include <tyndall/meta/hash.h>
#include <tyndall/meta/strval.h>

#ifndef IPC_SHMEM_PREFIX
#define IPC_SHMEM_PREFIX "tyn"
#endif

// Shared memory files all go in /dev/shm, so they need to be unique, and they
// can't have folders. We take names that look like absolute paths, like
// "/my-process/my_topic". We start with a prepend a constant unique prefix
// (IPC_SHMEM_PREFIX), Then we add the id, without leading slashes, and with the
// remaining slashes replaced with underscores.

// Note that "my-topic" gets the same id as "/my-topic" and "//my-topic" etc.

// Compile time id generation:
// Meant to take strval as template parameter.
template <typename STRING>
using id_remove_leading_slashes =
    decltype(STRING::template remove_leading<'/'>());

template <typename STRING>
using id_replace_slashes_with_underscores =
    decltype(STRING::template replace<'/', '%'>());

template <typename ID>
using id_prepare =
    decltype(create_strval(IPC_SHMEM_PREFIX) + "_"_strval +
             id_replace_slashes_with_underscores<
                 id_remove_leading_slashes<ID>>{}
    );

// runtime id generation
template <typename STORAGE>
static inline std::string id_rtid_prepare(const char *id) {
  // remove leading slashes
  while (*id == '/')
    ++id;

  std::string prepared_id = IPC_SHMEM_PREFIX "_" + std::string{id};

  // replace slash with underscore
  for (char &c : prepared_id)
    if (c == '/')
      c = '%';

  return prepared_id;
}
