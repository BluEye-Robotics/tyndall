#include <cstdio>
#include <tyndall/ros_context.h>
#ifdef NO_ROS
namespace std_msgs::msg {
struct Int32 {
  int data;
};
} // namespace std_msgs::msg
#else
#include <std_msgs/msg/int32.hpp>
#endif
#include <chrono>
#include <csignal>
#include <thread>

sig_atomic_t run = 1;

void signal_handler(int sig) { run = 0; }

int main(int argc, char **argv) {
  ros_context::init(argc, argv, std::chrono::milliseconds{3},
                    "ex_ros_context_read");

  signal(SIGINT, signal_handler);

  while (run) {
    std_msgs::msg::Int32 msg{};

    int rc = ros_context_read(msg, "/ex_ros_context");

    if (rc == 0)
      printf("read: %d\n", msg.data);

    std::this_thread::sleep_for(std::chrono::milliseconds{3});
  }

  return 0;
}
