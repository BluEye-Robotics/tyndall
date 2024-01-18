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
static inline int shutdown() { return 0; }
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
#include <rclcpp/rclcpp.hpp>

#include "tyndall/meta/strval.h"

// ros_context wraps ros initialization, destruction and pub sub pattern in a
// thread safe interface. lazy initialization of ros communication objects is
// used for ease of use
namespace ros_context {
extern rclcpp::Node::SharedPtr nh;
extern std::mutex ros_mutex;
extern std::thread ros_thread;
extern int run_ros;

// methods

static inline int shutdown() {
  run_ros = 0;
  ros_thread.join();

  rclcpp::shutdown();
  return 0;
}

static inline int
init(int argc, char **argv,
     std::chrono::milliseconds loop_sleep = std::chrono::milliseconds{3},
     const char *node_name = "default_node_name") {
  assert(nh == NULL); // enforce single initialization per process

  rclcpp::init(argc, argv);

  static rclcpp::Node::SharedPtr nh_new =
      std::make_shared<rclcpp::Node>(node_name);
  nh = nh_new;

  static struct lifetime_t {
    ~lifetime_t() { shutdown(); }
  } lifetime;

  ros_thread = std::thread([loop_sleep]() {
    while (run_ros) {
      {
        std::lock_guard<typeof(ros_mutex)> guard(ros_mutex);
        rclcpp::spin_some(nh);
      }
      std::this_thread::sleep_for(loop_sleep);
    }
  });
  return 0;
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
    std::lock_guard<typeof(ros_mutex)> guard(ros_mutex);

    static auto sub = nh->create_subscription<Message>(
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
void lazy_write(const Message &msg, Id) {
  static rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();
  static rclcpp::PublisherOptions pub_options;
  static auto pub =
      nh->create_publisher<Message>(Id::c_str(), qos_profile, pub_options);
  pub->publish(msg);
}

// Ros service based methods

template <typename Service, typename Id>
int lazy_serve(typename Service::Request &srv_req,
               typename Service::Response &srv_resp) {
  static typename Service::Request save_req;
  static typename Service::Response save_resp;
  static std::mutex save_mutex;
  static bool new_save = false;
  static bool valid_save = false;

  // handle service data
  int rc;
  {
    std::lock_guard<typeof(save_mutex)> guard(save_mutex);

    save_req = srv_req;
    save_resp = srv_resp; // set new response

    if (new_save) {
      new_save = false;
      srv_req = save_req;
      srv_resp = save_resp;
      rc = 0;
    } else if (valid_save) {
      srv_req = save_req;
      srv_resp = save_resp;
      rc = -1;
      errno = EAGAIN;
    } else {
      rc = -1;
      errno = ENOMSG;
    }
  }

  // register ros callback
  static bool must_initialize_callback = true;
  if (must_initialize_callback) {
    must_initialize_callback = false;
    std::lock_guard<typeof(ros_mutex)> guard(ros_mutex);

    static auto server = nh->create_service<Service>(
        Id::c_str(),
        std::function<bool(typename Service::Request::SharedPtr,
                           typename Service::Response::SharedPtr)>(
            [](typename Service::Request::SharedPtr req,
               typename Service::Response::SharedPtr rep) -> bool {
              std::lock_guard<typeof(save_mutex)> guard(save_mutex);
              *rep = save_resp;
              save_req = *req;
              new_save = true;
              valid_save = true;
              return true;
            }));
  }

  return rc;
}

template <typename Service, typename Id>
int lazy_call(typename Service::Request &srv_req,
              typename Service::Response &srv_resp, Id) {
  static auto client = nh->create_client<Service>(Id::c_str());

  if (!client.isValid()) {
    client = nh->create_client<Service>(Id::c_str());
  }
  auto result = client->async_send_request(srv_req);
  if (rclcpp::spin_until_future_complete(nh, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    srv_resp = result.get();
    return 0;
  }
  return 1;
}
} // namespace ros_context

#define ros_context_read(msg, id) ros_context::lazy_read(msg, id##_strval)

#define ros_context_write(msg, id) ros_context::lazy_write(msg, id##_strval)

template <typename Service, typename Id>
int ros_context_serve(typename Service::Request &srv_req,
                      typename Service::Response &srv_resp) {
  return ros_context::lazy_serve<Service, Id>(srv_req, srv_resp);
}

template <typename Service, typename Id>
int ros_context_serve(typename Service::Request &srv_req,
                      typename Service::Response &srv_resp, Id) {
  return ros_context::lazy_serve<Service, Id>(srv_req, srv_resp);
}

template <typename Request, typename Response>
int ros_context_call(Request &srv_req, Response &srv_resp, const char *id) {
  using Service = typename std::decay<decltype(srv_req)>::type;
  return ros_context::lazy_call<Service>(srv_req, srv_resp, strval(id));
}

#define ros_context_call(srv_req, srv_resp, id)                                \
  ros_context::lazy_call(srv_req, srv_resp, id##_strval)

#endif
