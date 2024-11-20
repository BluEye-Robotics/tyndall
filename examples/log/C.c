#define LOG_PRINTF
// #define LOG_FMT
#include <tyndall/log/log.h>

#include <signal.h>
#include <unistd.h>

sig_atomic_t run = 1;

void cb_sig(int sig) {
  log_info("got signal %d\n", sig);
  run = 0;
}

int main() {
  log_debug("hei %d\n", 3);

  signal(SIGINT, cb_sig);

  while (run) {
    log_debug("loopityloop\n");

    log_once_error("loopityloop\n");

    usleep(100000);
  }
}
