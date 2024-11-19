#pragma once
#include <google/protobuf/empty.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/wrappers.pb.h>
#include <zmq.h>

#include <errno.h>
#include <string.h>

#ifdef ZMQ_PROTO_DEBUG
#define zmq_proto_assert(expr)                                                 \
  do {                                                                         \
    (void)sizeof(expr);                                                        \
  } while (0)
#else
#define zmq_proto_assert(expr)                                                 \
  do {                                                                         \
    if (!(expr)) {                                                             \
      fprintf(stderr,                                                          \
              "[ %s:%d %s ] "                                                  \
              "assertion failed: (" #expr ") errno: %s\n",                     \
              __FILE__, __LINE__, __func__, zmq_strerror(zmq_errno()));        \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)
#endif

namespace zmq_proto {

// Wrapper types around zeromq contexts, sockets and messages, as per
// https://linux.die.net/man/7/zmq :

class context_t {
  void *context;

public:
  context_t(int n_threads) {
    int rc = init(n_threads);
    zmq_proto_assert(rc == 0);
  }

  int init(int n_threads) {
    context = zmq_ctx_new();
    zmq_proto_assert(context != NULL);

    {
      int rc = zmq_ctx_set(context, ZMQ_IO_THREADS, n_threads);
      zmq_proto_assert(rc == 0);
    }

    return 0;
  }

  void *zmq_handle() const { return context; }

  void *zmq_handle() { return context; }

  ~context_t() {
    int rc;
    do {
      rc = zmq_ctx_destroy(context);
    } while (rc == -1 && zmq_errno() == EINTR);

    zmq_proto_assert(rc == 0);
    context = NULL;
  }
};

enum socket_type_t {
  REQ = ZMQ_REQ,
  REP = ZMQ_REP,
  PUB = ZMQ_PUB,
  SUB = ZMQ_SUB
};

enum socket_mode_t { UNSPECIFIED, BIND, CONNECT };

template <socket_type_t socket_type> class socket_t {
  void *socket;

public:
  socket_t(const context_t &context, const char *addr,
           socket_mode_t mode = UNSPECIFIED) {
    socket = zmq_socket(context.zmq_handle(), static_cast<int>(socket_type));
    zmq_proto_assert(socket != NULL);

    {
      int rc;

      switch (mode) {
      case BIND:
        rc = zmq_bind(socket, addr);
        break;

      case CONNECT:
        rc = zmq_connect(socket, addr);
        break;

      case UNSPECIFIED:
        switch (socket_type) {
        case REP:
        case PUB:
          rc = zmq_bind(socket, addr);
          break;
        case REQ:
        case SUB:
          rc = zmq_connect(socket, addr);
          break;
        }
        break;
      }

      zmq_proto_assert(rc == 0);

      if (socket_type == SUB)
        rc = zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "", 0);

      zmq_proto_assert(rc == 0);
    }
  }

  void *zmq_handle() const { return socket; }

  void *zmq_handle() { return socket; }

  ~socket_t() {
    int rc = zmq_close(socket);
    zmq_proto_assert(rc == 0);

    socket = NULL;
  }
};

class msg_t {
  zmq_msg_t msg;

public:
  msg_t() {
    int rc = zmq_msg_init(&msg);
    zmq_proto_assert(rc == 0);
  }

  const zmq_msg_t *zmq_handle() const { return &msg; }

  zmq_msg_t *zmq_handle() { return &msg; }

  const char *data() { return static_cast<const char *>(zmq_msg_data(&msg)); }

  int size() const { return static_cast<int>(zmq_msg_size(&msg)); }

  ~msg_t() {
    int rc = zmq_msg_close(&msg);
    zmq_proto_assert(rc == 0);
  }
};

// helpers:

static inline int check_proto_type(msg_t &msg,
                                   const std::string &proto_type_name) {
  if (strncmp(msg.data(), proto_type_name.data(), proto_type_name.size()) == 0)
    return 0;
  else {
    errno = EBADMSG;
    return -1;
  }
}

template <typename Message>
  requires(std::is_base_of<::google::protobuf::Message, Message>::value)
int check_proto_type(msg_t &msg) {
  std::string type_name =
      Message{}
          .GetTypeName(); // Beware of unnecessary instantiation:
                          // https://github.com/protocolbuffers/protobuf/issues/2573
  return check_proto_type(msg, type_name);
}

template <typename Message>
  requires(std::is_base_of<::google::protobuf::Message, Message>::value)
const char *get_debug_string(const Message &proto) {
  static std::string debug_string;
  debug_string = proto.GetTypeName() + " { " + proto.ShortDebugString() + " }";
  return debug_string.c_str();
}

// Send and receive functions for zeromq messages with protobuf payloads:

// A message consists of two frames.
// First frame is the type name of the protobuf
// Second frame is a serialized protobuf.

enum send_recv_flags {
  NONE = 0,
  DONTWAIT = ZMQ_DONTWAIT,
  SNDMORE = ZMQ_SNDMORE
};

/*
  sends two-part message. first is type name, second is proto
  errno: EAGAIN if the outbound message queue was full
*/
template <socket_type_t socket_type>
int send(const ::google::protobuf::MessageLite &payload,
         const socket_t<socket_type> &socket, send_recv_flags flags = NONE) {
  int rc;

  std::string type_name = payload.GetTypeName();

  // first send type name
  const int sent =
      zmq_send(socket.zmq_handle(), type_name.data(), type_name.size(),
               static_cast<int>(flags | SNDMORE));
  if (sent < 0) {
    zmq_proto_assert(zmq_errno() == EAGAIN);
    rc = -1;
    errno = EAGAIN;
  } else {
    const int payload_size = static_cast<int>(payload.ByteSizeLong());
    char payload_buf[payload_size];
    bool ser_rc = payload.SerializeToArray(payload_buf, payload_size);
    zmq_proto_assert(ser_rc);

    // then send payload
    const int sent = zmq_send(socket.zmq_handle(), payload_buf, payload_size,
                              static_cast<int>(flags));
    if (sent < 0) {
      zmq_proto_assert(zmq_errno() == EAGAIN);
      rc = -1;
      errno = EAGAIN;
    } else
      rc = 0;
  }

  return rc;
}

/*
  errno: EAGAIN if the inbound message queue was empty
*/
template <socket_type_t socket_type>
int recv(msg_t &msg, const socket_t<socket_type> &socket,
         send_recv_flags flags = NONE) {
  int rc;

  const int received = zmq_msg_recv(msg.zmq_handle(), socket.zmq_handle(),
                                    static_cast<int>(flags));

  if (received < 0) {
    zmq_proto_assert(zmq_errno() == EAGAIN);
    rc = -1;
    errno = EAGAIN;
  } else {
    rc = 0;
    zmq_proto_assert(msg.size() == received);
  }

  return rc;
}

/*
  receives another part of a multipart message
  errno: EOPNOTSUPP if there were no more message parts to receive
*/
template <socket_type_t socket_type>
int recv_more(msg_t &msg, const socket_t<socket_type> &socket,
              send_recv_flags flags = NONE) {
  int64_t more;
  size_t more_size = sizeof(more);

  int rc = zmq_getsockopt(socket.zmq_handle(), ZMQ_RCVMORE, &more, &more_size);
  zmq_proto_assert(rc == 0);

  if (more == 1) {
    rc = recv(msg, socket, flags);
  } else {
    assert(more == 0);
    rc = -1;
    errno = EOPNOTSUPP;
  }

  return rc;
}

/*
  receives all remaining parts of a multipart message
  errno: EOPNOTSUPP
*/
template <socket_type_t socket_type>
int recv_more(const socket_t<socket_type> &socket,
              send_recv_flags flags = NONE) {
  int rc;
  do {
    msg_t msg;
    rc = recv_more(msg, socket, flags);
  } while (rc == 0);

  return rc;
}

/*
  receives another part of a multipart message, and parses into the proto
  message errno: EOPNOTSUPP if there were no more message parts to receive
  errno: EPROTO if the proto could not be parsed
*/
template <socket_type_t socket_type>
int recv_more(::google::protobuf::MessageLite &proto_msg,
              const socket_t<socket_type> &socket,
              send_recv_flags flags = NONE) {
  msg_t msg;

  int rc = recv_more(msg, socket, flags);

  if (rc != 0)
    rc = -1;
  else if (!proto_msg.ParseFromArray(msg.data(), msg.size())) {
    rc = -1;
    errno = EPROTO;
  } else
    rc = 0;

  return rc;
}

/*
  receives two parts of a multipart message. Matches first with type name and
  parses second into proto errno: EAGAIN if the inbound message queue was empty
  errno: EBADMSG if the type name didn't match
  errno: EOPNOTSUPP if there were no more message parts to receive after first
  part errno: EPROTO if the proto could not be parsed / did not match
*/
template <socket_type_t socket_type>
int recv(::google::protobuf::MessageLite &proto_msg,
         const socket_t<socket_type> &socket, send_recv_flags flags = NONE) {
  msg_t type_name_msg;

  int rc;
  (0 == (rc = recv(type_name_msg, socket, flags))) &&
      (0 == (rc = check_proto_type(type_name_msg, proto_msg.GetTypeName()))) &&
      (0 == (rc = recv_more(proto_msg, socket, flags)));

  return rc;
}

/*
  receives two parts of a multipart message and discards them
  errno: EAGAIN if the inbound message queue was empty
  errno: EOPNOTSUPP if there were no more message parts to receive after first
  part
*/
template <socket_type_t socket_type>
int recv(const socket_t<socket_type> &socket, send_recv_flags flags = NONE) {
  msg_t msg;

  int rc;
  (0 == (rc = recv(msg, socket, flags))) &&
      (0 == (rc = recv_more(socket, flags)));

  return rc;
}

} // namespace zmq_proto
