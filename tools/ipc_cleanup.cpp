#include <dirent.h>
#include <stdio.h>
#include <tyndall/ipc/ipc.h>
#include <tyndall/ipc/shmem.h>

int main() {
  const char *prefix = IPC_SHMEM_PREFIX;

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

    if (match) {
      printf("removing %s\n", dir_name);
      shmem_unlink(dir_name);
    }
  }

  closedir(dir);

  return 0;
}
