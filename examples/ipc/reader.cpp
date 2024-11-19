#include <signal.h>

#include <tyndall/ipc/ipc.h>
#define debug_stdout stdout
#define debug_stderr stderr
#include <tyndall/debug.h>

#include "my_struct.h"

int run = 1;

void sig_handler(int sig) { run = 0; }

int main() {
  signal(SIGINT, sig_handler);

  while (run) {
    my_struct entry;

    if (ipc_read(entry, "/my/topic") == 0) {
      debug("new entry:\t");
      my_struct_print(debug_plain, entry, "\n");
    } else
      debug_error("no entry\n");

    usleep(3 * 1000);
  }

  return 0;
}
