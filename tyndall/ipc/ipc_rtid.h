#pragma once
#include <vector>

#include "id.h"
#include "seq_lock.h"
#include "shmem.h"
#include <tyndall/meta/typeinfo.h>

// Implementation of ipc structs and methods from ipc.h, but with id specified
// at runtime. The normal, non-runtime implementation is preferred since it is
// cleaner.

template <typename STORAGE>
using ipc_rtid_writer = shmem_buf<seq_lock<STORAGE>, SHMEM_WRITE>;
template <typename STORAGE>
using ipc_rtid_reader = shmem_buf<seq_lock<STORAGE>, SHMEM_READ>;

// Return and errno values are the same as for ipc_write / read
#define ipc_rtid_write(entry, id)                                              \
  create_ipc_rtid_lazy<ipc_rtid_writer, std::remove_cvref_t<decltype(entry)>>( \
      id)                                                                      \
      .write(entry)
#define ipc_rtid_read(entry, id)                                               \
  create_ipc_rtid_lazy<ipc_rtid_reader, std::remove_cvref_t<decltype(entry)>>( \
      id)                                                                      \
      .read(entry)

template <typename STORAGE>
static inline ipc_rtid_writer<STORAGE> create_ipc_rtid_writer(const char *id) {
  std::string prepared_id = id_rtid_prepare<STORAGE>(id);
  return ipc_rtid_writer<STORAGE>{prepared_id.c_str()};
}

template <typename STORAGE>
static inline ipc_rtid_reader<STORAGE> create_ipc_rtid_reader(const char *id) {
  std::string prepared_id = id_rtid_prepare<STORAGE>(id);
  return ipc_rtid_reader<STORAGE>{prepared_id.c_str()};
}

// To keep track of lazily created ipc objects, the runtime id needs to be
// looked up in a registry. You might want to keep track of it manually instead.
template <template <typename> typename TRANSPORT, typename STORAGE>
static inline TRANSPORT<STORAGE> &create_ipc_rtid_lazy(const char *id) {
  std::string prepared_id = id_rtid_prepare<STORAGE>(id);

  // manual registry is needed as substitution for compile time template
  // matching
  struct registry_item {
    std::string id;
    TRANSPORT<STORAGE> transport;
  };
  static std::vector<registry_item> registry;

  for (auto &item : registry)
    if (item.id == prepared_id)
      return item.transport;

  registry.push_back(registry_item{
      .id = prepared_id, .transport = TRANSPORT<STORAGE>{prepared_id.c_str()}});

  return registry.back().transport;
}
