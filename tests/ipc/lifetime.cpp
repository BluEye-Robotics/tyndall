// Creating and destroying IPC endpoints must not leave file descriptors or
// shared memory mappings behind: a process that recreates its writers (a
// recorder with one fpscalc element per recording) would run out of fds.
// Linux only: counts /proc/self/fd and /proc/self/maps.
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tyndall/ipc/ipc.h>
#include <tyndall/meta/macro.h>

#define check(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      ipc_cleanup();                                                           \
      printf(__FILE__ ":" M_STRINGIFY(__LINE__) " "                            \
                                                "Assertion failed: " #cond     \
                                                "\n");                         \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)

// Open file descriptors of this process, or -1 without /proc.
static int open_fds() {
  DIR *dir = opendir("/proc/self/fd");
  if (dir == nullptr)
    return -1;
  int n = 0;
  while (readdir(dir) != nullptr)
    n++;
  closedir(dir);
  return n;
}

// Mappings of shared memory objects whose name contains `part`.
static int mappings(const char *part) {
  FILE *maps = fopen("/proc/self/maps", "r");
  if (maps == nullptr)
    return -1;
  char line[512];
  int n = 0;
  while (fgets(line, sizeof line, maps) != nullptr)
    if (strstr(line, "/dev/shm/") != nullptr && strstr(line, part) != nullptr)
      n++;
  fclose(maps);
  return n;
}

int main() {
  if (open_fds() == -1) {
    printf("lifetime: no /proc/self/fd, skipped\n");
    return 0;
  }
  ipc_cleanup();

  // The first endpoint may set up state that stays for the process (a
  // registry, a cached page): take the baseline after one round.
  {
    auto writer = create_ipc_rtid_writer<double>("/test/lifetime");
    writer.write(1.0);
  }
  const int fds_before = open_fds();
  const int maps_before = mappings("lifetime");

  for (int i = 0; i < 200; i++) {
    auto writer = create_ipc_rtid_writer<double>("/test/lifetime");
    double value = i;
    writer.write(value);
    auto reader = create_ipc_rtid_reader<double>("/test/lifetime");
    double read_back = -1;
    check(reader.read(read_back) == 0);
    check(read_back == value);
  }

  const int fds_after = open_fds();
  const int maps_after = mappings("lifetime");
  printf("lifetime: fds %d -> %d, mappings %d -> %d\n", fds_before, fds_after,
         maps_before, maps_after);
  check(fds_after == fds_before);
  check(maps_after == maps_before);

  ipc_cleanup();
  return 0;
}
