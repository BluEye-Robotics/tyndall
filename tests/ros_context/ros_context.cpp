#include <assert.h>
#include <sys/types.h>
#include <tyndall/meta/macro.h>
#include <tyndall/ros_context.h>
#include <unistd.h>
#ifdef NO_ROS
namespace std_msgs::msg {
struct Int32 {
  int data;
};
} // namespace std_msgs::msg
#else
#include <std_msgs/msg/int32.hpp>
#endif

using i32 = std_msgs::msg::Int32;

bool operator==(const i32 &lhs, const i32 &rhs) { return lhs.data == rhs.data; }

const i32 ref = []() {
  i32 ret;
  ret.data = 42;
  return ret;
}();

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
  ros_context::init(0, NULL, std::chrono::milliseconds{3}, "test_ros_context");

  // Create a thread to run ros_context::spin
  std::thread spin_thread([]() { ros_context::spin(); });

  {
    {
      i32 entry = ref;
      ros_context_write(entry, "/test/standard");
    }

    {
      i32 entry, tmp;
      ros_context_read(tmp, "/test/standard");
      sleep(1);
      int rc = ros_context_read(entry, "/test/standard");
      check((rc == 0) || (errno == EAGAIN));
      check(entry == ref);
    }
  }
  rclcpp::shutdown();
  spin_thread.join();

#ifdef NO_ROS
  ipc_cleanup();
#endif
}
