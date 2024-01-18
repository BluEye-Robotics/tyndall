#ifdef NO_ROS
#include <cstdio>
int main() { printf("no ros\n"); }
#else
#include <chrono>
#include <csignal>
#include <cstdio>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
#include <tyndall/ros_context.h>

sig_atomic_t run = 1;

void signal_handler(int sig) { run = 0; }

int main(int argc, char **argv) {
  ros_context::init(argc, argv, std::chrono::milliseconds{3},
                    "ex_ros_context_call");

  signal(SIGINT, signal_handler);

  while (run) {
    std_srvs::srv::SetBool::Request srv_req;
    std_srvs::srv::SetBool::Response srv_resp;
    srv_req.data = (bool)(run++ % 3);

    int rc = ros_context_call(srv_req, srv_resp,
                              "/ex_ros_context_serve/ex_ros_context");

    if (rc == 0)
      printf("req rep: %d %d\n", srv_req.data, srv_resp.success);
    else
      printf("response error\n");

    std::this_thread::sleep_for(std::chrono::milliseconds{3});
  }

  return 0;
}
#endif
