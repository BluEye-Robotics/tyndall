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
                    "ex_ros_context_serve");

  signal(SIGINT, signal_handler);

  while (run) {
    std_srvs::SetBool srv;
    srv.response.success =
        (bool)(std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count() %
               3);

    int rc = ros_context_serve(srv, "ex_ros_context");

    if (rc == 0)
      printf("read: %d\n", srv.request.data);

    std::this_thread::sleep_for(std::chrono::milliseconds{3});
  }

  return 0;
}
#endif
