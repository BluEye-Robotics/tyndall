#include <csignal>
#include <cstdio>
#include <google/protobuf/wrappers.pb.h>
#include <thread>
#include <tyndall/proto/binlog.h>

sig_atomic_t run = 1;

void signal_handler(int sig) { run = 0; }

int main() {
  int binlog_fd = binlog_create("log.bin");
  if (binlog_fd == -1) {
    printf("errno: %s\n", strerror(errno));
    exit(1);
  }

  for (int i = 0; run; ++i) {
    google::protobuf::Int32Value msg;
    msg.set_value(i);

    printf("logging value: %d\n", msg.value());

    int rc = binlog_write(binlog_fd, msg);
    if (rc != 0) {
      printf("errno: %s\n", strerror(errno));
      exit(1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
  }

  return 0;
}
