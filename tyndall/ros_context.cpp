
#include "ros_context.h"

#ifndef NO_ROS
namespace ros_context {
std::weak_ptr<rclcpp::Node> nh;
std::mutex ros_mutex;
std::thread ros_thread;
int run_ros = 1;
} // namespace ros_context
#endif
