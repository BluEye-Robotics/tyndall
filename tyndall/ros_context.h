#pragma once
#include <cerrno>
#include <chrono>
#include <mutex>
#include <optional>
#include <stdint.h>
#include <thread>

#ifdef NO_ROS
// mock
namespace ros_context {
static inline void spin() {}
static inline int
init(int argc, char **argv,
     std::chrono::milliseconds loop_sleep = std::chrono::milliseconds{3},
     const char *node_name = "default_node_name") {
  return 0;
}
} // namespace ros_context
#include <tyndall/ipc/ipc.h>
#define ros_context_read(...) ({ ipc_read(__VA_ARGS__); })
#define ros_context_write(...) ({ ipc_write(__VA_ARGS__); })
#define ros_context_serve(...)                                                 \
  ({                                                                           \
    (void)sizeof(msg);                                                         \
    0;                                                                         \
  })
#define ros_context_call(...)                                                  \
  ({                                                                           \
    (void)sizeof(msg);                                                         \
    0;                                                                         \
  })
#else
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tyndall/meta/strval.h"

// ros_context wraps ros initialization, destruction and pub sub pattern in a
// thread safe interface. lazy initialization of ros communication objects is
// used for ease of use
namespace ros_context {
extern std::weak_ptr<rclcpp::Node> nh;
extern std::mutex ros_mutex;

static inline int
init(int argc, char **argv,
     std::chrono::milliseconds loop_sleep = std::chrono::milliseconds{3},
     const char *node_name = "default_node_name") {
  // assert(nh == NULL); // enforce single initialization per process

  rclcpp::init(argc, argv);

  static rclcpp::Node::SharedPtr nh_new =
      std::make_shared<rclcpp::Node>(node_name);
  nh = nh_new;

  return 0;
}

static inline void spin() {
  rclcpp::experimental::executors::EventsExecutor executor;

  // Add the node to the executor
  executor.add_node(nh.lock());

  // Spin the executor
  executor.spin();

  // Remove the node from the executor after spinning
  executor.remove_node(nh.lock());
}

template <typename Message, typename Id> int lazy_read(Message &msg, Id) {
  static Message save;
  static std::mutex save_mutex;
  static bool new_save = false;
  static bool valid_save = false;

  // register ros callback
  static bool must_initialize_callback = true;
  if (must_initialize_callback) {
    must_initialize_callback = false;
    // std::lock_guard<typeof(ros_mutex)> guard(ros_mutex);
    static auto sub = nh.lock()->create_subscription<Message>(
        Id::c_str(), 1,
        std::function<void(const typename Message::ConstSharedPtr)>(
            [](const typename Message::ConstSharedPtr sub_msg) -> void {
              std::lock_guard<typeof(save_mutex)> guard(save_mutex);
              save = *sub_msg;
              new_save = true;
              valid_save = true;
            }));
  }

  // get saved ros message
  int rc;
  {
    std::lock_guard<typeof(save_mutex)> guard(save_mutex);
    if (new_save) {
      new_save = false;
      msg = save;
      rc = 0;
    } else if (valid_save) {
      msg = save;
      rc = -1;
      errno = EAGAIN;
    } else {
      rc = -1;
      errno = ENOMSG;
    }
  }

  return rc;
}

template <typename Message, typename Id>
static rclcpp::Publisher<Message>::SharedPtr get_publisher() {
  std::lock_guard<typeof(ros_mutex)> guard(ros_mutex);
  static rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();
  static rclcpp::PublisherOptions pub_options;
  if (auto n = nh.lock()) {
    return n->create_publisher<Message>(Id::c_str(), qos_profile, pub_options);
  }
  throw std::runtime_error("ros_context::get_publisher: node is not valid");
}

template <typename Message, typename Id>
void lazy_write(const Message &msg, Id) {

  try {
    static auto pub = get_publisher<Message, Id>();
    pub->publish(msg);
  } catch (const std::exception &e) {
    std::cerr << "ros_context::lazy_write: " << e.what() << std::endl;
  }
}

template <typename Service, typename Id>
static rclcpp::Client<Service>::SharedPtr get_client() {
  std::lock_guard<typeof(ros_mutex)> guard(ros_mutex);
  if (auto n = nh.lock()) {
    std::cout << "ros_context::lazy_call: Starting service client for "
              << Id::c_str() << std::endl;
    return n->create_client<Service>(Id::c_str());
  }
  throw std::runtime_error("ros_context::get_client: node is not valid");
}

template <typename Service, typename Id>
typename rclcpp::Client<Service>::SharedResponse
lazy_call(typename rclcpp::Client<Service>::SharedRequest &req, const Id &) {
  try {
    static auto client = get_client<Service, Id>();
    std::mutex m;
    std::condition_variable cv;
    std::unique_lock lk(m);
    bool ready = false;
    typename rclcpp::Client<Service>::SharedResponse response = nullptr;

    auto result = client->async_send_request(
        req, [&](typename rclcpp::Client<Service>::SharedFuture future) {
          ready = true;
          response = future.get();
          cv.notify_all();
          if (future.get()) {
            std::cout << "ros_context::lazy_call: Service call succeeded"
                      << std::endl;
          } else {
            std::cerr << "ros_context::lazy_call: Service call failed"
                      << std::endl;
          }
        });

    cv.wait_for(lk, std::chrono::seconds(5), [&] { return ready; });

    if (!ready) {
      std::cerr << "ros_context::lazy_call: Service call timed out"
                << std::endl;
      return {};
    }

    return response;
  } catch (const std::exception &e) {
    std::cerr << "ros_context::lazy_call: " << e.what() << std::endl;
  }
  return {};
}

} // namespace ros_context

#define ros_context_read(msg, id) ros_context::lazy_read(msg, id##_strval)

#define ros_context_write(msg, id) ros_context::lazy_write(msg, id##_strval)

#define ros_context_call(Service, req, id)                                     \
  ros_context::lazy_call<Service>(req, id##_strval)

#endif
