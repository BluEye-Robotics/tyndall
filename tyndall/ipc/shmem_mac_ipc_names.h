#pragma once

#ifdef __APPLE__

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <fstream>
#include <set>
#include <cstring>
#include <iostream>

static const char *ipc_names_file = "/tmp/tyndall_ipc_names";

inline void append_ipc_name(const std::string &name) {
  int fd = open(ipc_names_file, O_RDWR | O_CREAT, 0666);
  if (fd == -1) {
    perror("open ipc_names_file");
    return;
  }
  if (flock(fd, LOCK_EX) == -1) {
    perror("flock");
    close(fd);
    return;
  }

  std::set<std::string> names;
  {
    std::ifstream infile(ipc_names_file);
    std::string line;
    while (std::getline(infile, line)) {
      names.insert(line);
    }
  }

  if (names.find(name) == names.end()) {
    names.insert(name);
    std::ofstream outfile(ipc_names_file, std::ios::trunc);
    for (const auto &n : names) {
      outfile << n << "\n";
    }
  }

  flock(fd, LOCK_UN);
  close(fd);
}

inline int unlink_all_ipc_names() {
  int fd = open(ipc_names_file, O_RDWR);
  if (fd == -1) {
    return 0; // no file, nothing to unlink
  }

  if (flock(fd, LOCK_EX) == -1) {
    perror("flock");
    close(fd);
    return -1;
  }

  std::set<std::string> names_to_unlink;
  std::set<std::string> names_to_keep;

  {
    std::ifstream infile(ipc_names_file);
    std::string line;
    while (std::getline(infile, line)) {
        names_to_unlink.insert(line);
    }
  }

  for (const auto &name : names_to_unlink) {
    if (shm_unlink(name.c_str()) != 0) {
      perror(("shm_unlink " + name).c_str());
    }
  }

  {
    std::ofstream outfile(ipc_names_file, std::ios::trunc);
    for (const auto &n : names_to_keep) {
      outfile << n << "\n";
    }
  }

  flock(fd, LOCK_UN);
  close(fd);
  return 0;
}

#endif // __APPLE__
