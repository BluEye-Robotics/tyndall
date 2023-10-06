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
    std_srvs::SetBool srv;
    srv.request.data = (bool)(run++ % 3);

    int rc = ros_context_call(srv, "/ex_ros_context_serve/ex_ros_context");

    if (rc == 0)
      printf("req rep: %d %d\n", srv.request.data, srv.response.success);
    else
      printf("response error\n");

    std::this_thread::sleep_for(std::chrono::milliseconds{3});
  }

  return 0;
}
#endif
