#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
// #include <tyndall/ext/embed.hpp>
// #include <experimental/reflect>
#include <ctype.h>
#include <string.h>
#include <typeinfo>

// ipc
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <tyndall/ipc/seq_lock.h>
#include <tyndall/reflect/print_format.h>
#include <unistd.h>

void print(char *fmt, const char *buf, size_t buf_size) {
  const char *b = buf;
  for (char *f = fmt; (*f != '\0'); ++f) {
    if (isdigit(*f)) {
      long num = strtol(f, &f, 10);
      printf("<%ld>, ", num);
      b += num;
    }

    size_t printed = print_format(*f, b);
    if (printed == 0) {
      switch (*f) {
      case 's':
        printf("%ss, ", b);
        printed = strlen(b);
        b += printed;
        break;
      default:
        printf("error, wrong parameter: %c\n", *f);
      }
      if (printed == 0)
        break;
    } else {
      printf(",\t");
      b += printed;
    }

    if ((size_t)(b - buf) > buf_size) {
      printf("fmt is too large for buffer at %zu", buf_size);
      break;
    }
  }
  printf("\n");
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Usage: %s <ipc_name> <format>\n", argv[0]);
    exit(1);
  }

  const char *ipc_id = argv[1];
  const char *ipc_prefix = "/dev/shm/";

  if (strncmp(ipc_id, ipc_prefix, strlen(ipc_prefix)) == 0)
    ipc_id += strlen(ipc_prefix);

  // printf("id: %s\n", ipc_id);

  // ipc
  int fd = shm_open(ipc_id, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
  assert(fd != -1);

  size_t ipc_size;
  {
    struct stat st;
    int rc = fstat(fd, &st);
    assert(rc == 0);
    ipc_size = st.st_size;
  }

  void *const mapped =
      mmap(NULL, ipc_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  assert(mapped != MAP_FAILED);
  assert(reinterpret_cast<uintptr_t>(mapped) % CACHELINE_BYTES == 0);

  unsigned *ipc_seq = (unsigned *)mapped;
  const char *const ipc_buf = (const char *)mapped + CACHELINE_BYTES;
  const size_t buf_size = ipc_size - 2 * sizeof(int) - CACHELINE_BYTES;
  char *buf = (char *)malloc(buf_size + CACHELINE_BYTES);
  char *const buf_to_free = buf;

  // align buf
  {
    constexpr size_t alignment = CACHELINE_BYTES;
    size_t alignment_error = reinterpret_cast<uintptr_t>(buf) % alignment;
    if (alignment_error > 0)
      buf += alignment - alignment_error;
  }

  unsigned seq = 0;
  while (1) {
    bool new_buf = false;
    {

      unsigned seq1;
      do {
        seq1 = seq_lock_read_begin(ipc_seq);
        memcpy(buf, ipc_buf, buf_size);

      } while (seq_lock_read_retry(ipc_seq, seq1));

      if (seq1 != seq) {
        new_buf = true;
        seq = seq1;
      }
    }

    if (new_buf) {
      if (argc > 2) {
        char *fmt = argv[2];
        print(fmt, buf, buf_size);
      } else {
        // extract debug format
        constexpr int type_info_hash_size = 4;
        const size_t debug_tail_size = ipc_size % CACHELINE_BYTES;

        if (debug_tail_size > type_info_hash_size) {
          const size_t debug_format_size =
              debug_tail_size - type_info_hash_size;
          char *const debug_format_loc =
              static_cast<char *>(mapped) + ipc_size - debug_format_size;

          char *fmt = debug_format_loc;
          print(fmt, buf, buf_size);
        } else {
          for (size_t i = 0; i < buf_size; ++i)
            printf("%02x", buf[i]);
          printf("\n");
          // printf("int: %d\n", *(int*)buf);
        }
      }
    } else
      usleep(1000);
  }

  free(buf_to_free);

  {
    int rc = munmap(mapped, ipc_size);
    assert(rc == 0);
  }

  return 0;
}
