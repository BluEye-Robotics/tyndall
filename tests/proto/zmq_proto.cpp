#include <assert.h>
#include <google/protobuf/wrappers.pb.h>
#include <thread>
#include <tyndall/meta/macro.h>
#include <tyndall/proto/zmq_proto.h>

#define check(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf(__FILE__ ":" M_STRINGIFY(__LINE__) " "                            \
                                                "Assertion failed: " #cond     \
                                                "\n");                         \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)

int main() {
  zmq_proto::context_t context{1};
  zmq_proto::socket_t<zmq_proto::PUB> pub(context, "tcp://*:5444");
  zmq_proto::socket_t<zmq_proto::SUB> sub(context, "tcp://127.0.0.1:5444");

  std::thread sub_thread([&sub]() {
    google::protobuf::Int32Value msg;
    msg.set_value(0);

    int rc = zmq_proto::recv(msg, sub);
    check(rc == 0);
    check(msg.value() == 42);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds{50});

  std::thread pub_thread([&pub]() {
    google::protobuf::Int32Value msg;
    msg.set_value(42);

    int rc = zmq_proto::send(msg, pub);
    check(rc == 0);
  });

  pub_thread.join();
  sub_thread.join();
}
