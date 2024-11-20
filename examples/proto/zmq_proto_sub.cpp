#include <chrono>
#include <csignal>
#include <cstdio>
#include <google/protobuf/wrappers.pb.h>
#include <thread>
#include <tyndall/proto/zmq_proto.h>

sig_atomic_t run = 1;

void signal_handler(int sig) { run = 0; }

int main() {
  zmq_proto::context_t context{1};
  zmq_proto::socket_t<zmq_proto::SUB> socket(context, "tcp://127.0.0.1:5444");

  signal(SIGINT, signal_handler);

  while (run) {
    google::protobuf::Int32Value msg;

    int rc = zmq_proto::recv(msg, socket);
    if (rc != 0) {
      printf("errno: %s\n", strerror(errno));
      exit(1);
    }

    printf("got value: %d\n", msg.value());

    std::this_thread::sleep_for(std::chrono::milliseconds{3});
  }

  return 0;
}
