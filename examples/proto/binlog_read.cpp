#include <cstdio>
#include <google/protobuf/wrappers.pb.h>
#include <thread>
#include <tyndall/proto/binlog.h>

int main() {
  int binlog_fd = binlog_open("log.bin");
  if (binlog_fd == -1) {
    printf("errno: %s\n", strerror(errno));
    exit(1);
  }

  while (1) {
    google::protobuf::Int32Value msg;

    int rc = binlog_read(binlog_fd, msg);
    if (rc != 0) {
      printf("errno: %s\n", strerror(errno));
      exit(1);
    }

    printf("got value: %d\n", msg.value());

    std::this_thread::sleep_for(std::chrono::milliseconds{100});
  }

  return 0;
}
