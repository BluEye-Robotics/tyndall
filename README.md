# Tyndall

[![tests](https://github.com/BluEye-Robotics/tyndall/actions/workflows/tests.yml/badge.svg)](https://github.com/BluEye-Robotics/tyndall/actions)

Blueye Robotics open source c++ components

### Requirements

gcc / c++20, cmake

[ZeroMQ/libzmq](https://github.com/zeromq/libzmq)

[Protobuf (c++)](https://github.com/protocolbuffers/protobuf)

[fmt](https://github.com/fmtlib/fmt)

[Boost.Python](https://github.com/boostorg/python)

[ROS (optional)](http://wiki.ros.org/ROS/Tutorials)

### Build
```
mkdir build
cd build
cmake ..
make
```

### Install
```
make install
```

### Build examples
```
make examples
```

### Build and run tests
```
make tests
```
(Tests won't get executed when cross compiling.)

# Highlights

## meta/iterate

Function that calls the given callback with integral constants from 0 to the specified index.
Useful for iterating at compile time.

Example:
```cpp
#include <tyndall/meta/iterate.h>

iterate<10>(
[](auto index){
  // index = std::integral_constant<int, 0>(), std::integral_constant<int, 1>(), ... ,std::integral_constant<int, 9>()
  static_assert((index >= 0) && (index < 10));
  printf("%d ", index()); // 0 1 2 3 4 5 6 7 8 9
});
```

## meta/strval

compile time string which ensures "string A"\_strval and "string A"\_strval have the same type,
while "string A"\_strval and "string B"\_strval have different types.

Example:
```cpp
constexpr auto s = "hei"_strval;
printf("%s\n", s.c_str());
printf("%s\n", decltype(sv)::c_str());
printf("%s\n", ("belle"_strval).replace<'e', 'a'>().c_str());
printf("%s\n", to_strval<42>::c_str());
```

## meta/typevals
Container where entries can have different types.

Example:
```cpp
#include <tyndall/meta/typevals.h>

struct A{int a; float b;};
struct B{double x; char y; char z;};

static constexpr auto tv = typevals{} + 5 + A{4,2} + B{3,9,1};
static_assert(tv.size() == 3);

constexpr auto A42 = tv.get<1>();
static_assert(A42.b == 2);
constexpr auto B391 = tv[std::integral_constant<int, 2>()];
static_assert(B391.x == 3);
```

## zmq\_proto

[ZeroMQ](https://zeromq.org/) ([libzmq](https://github.com/zeromq/libzmq)) / [Protobuf](https://developers.google.com/protocol-buffers) based tcp communication.

Example:

### Publisher
```cpp
#include <tyndall/proto/zmq_proto.h>

zmq_proto::context_t context{1};
zmq_proto::socket_t<zmq_proto::PUB> socket(context, "tcp://*:5444");

google::protobuf::Int32Value msg;
msg.set_value(42);

int rc = zmq_proto::send(msg, socket);
if (rc != 0)
  printf("errno: %s\n", strerror(errno));
```

### Subscriber
```cpp
#include <tyndall/proto/zmq_proto.h>

zmq_proto::context_t context{1};
zmq_proto::socket_t<zmq_proto::SUB> socket(context, "tcp://127.0.0.1:5444");

while(1)
{
  google::protobuf::Int32Value msg;

  int rc = zmq_proto::recv(msg, socket);
  if (rc != 0)
  {
    printf("errno: %s\n", strerror(errno));
    exit(1);
  }
}
```
Full examples in [examples/proto/zmq\_proto\_pub.cpp](examples/proto/zmq_proto_pub.cpp) and [examples/proto/zmq\_proto\_sub.cpp](examples/proto/zmq_proto_sub.cpp).

## ros\_context
A wrapper around ros.
It provides a minimal of lazy methods for sending and receiving ros messages and service calls.

Pub / sub example:

### Reader

```cpp
#include <tyndall/ros_context.h>
#include <std_msgs/msg/int32.h>

ros_context::init(argc, argv, std::chrono::milliseconds{3}, "ex_ros_context_write");

std_msgs::msg::Int32 msg;
msg.data = 42;

ros_context_write(msg, "/ex_ros_context");
```

### Writer

```cpp
#include <tyndall/ros_context.h>
#include <std_msgs/msg/int32.h>

ros_context::init(argc, argv, std::chrono::milliseconds{3}, "ex_ros_context_read");

while(1)
{
  std_msgs::msg::Int32 msg;

  int rc = ros_context_read(msg, "/ex_ros_context");

  if (rc == 0)
    printf("read: %d\n", msg.data);
}
```

Serve / call example:

### Server

```cpp
#include <tyndall/ros_context.h>
#include <std_srvs/srv/set_bool.hpp>

ros_context::init(argc, argv, std::chrono::milliseconds{3}, "ex_ros_context_serve");
while(1)
{
  std_srvs::srv::SetBool srv;
  srv.response.success = true;

  int rc = ros_context_serve(srv, "ex_ros_context");

  if (rc == 0)
    printf("got: %d\n", srv.request.data);
}
```

### Caller

```cpp
#include <tyndall/ros_context.h>
#include <std_srvs/srv/set_bool.hpp>

ros_context::init(argc, argv, std::chrono::milliseconds{3}, "ex_ros_context_serve");
while(1)
{
  std_srvs::srv::SetBool srv;
  srv.response.success = true;

  std_srvs::srv::SetBool srv;
  srv.request.data = true;

  int rc = ros_context_call(srv, "/ex_ros_context_serve/ex_ros_context");

  if (rc == 0)
    printf("got: %d\n", srv.response.success);
}
```

## ipc
Inter process communication in Linux based on shared memory and lockless data structures.

Shared memory is created (if needed) and mapped in `ipc_write` and `ipc_read` on the first call with a new combination of entry type and id.

Lockless [seqlock](https://en.wikipedia.org/wiki/Seqlock) is used for synchronization.
It supports a single writer and multiple readers.

Shared memory is left open on shutdown, so it gets reused on restart.
You can remove the shared memory using `ipc_cleanup();` or `rm /dev/shm/ipc*`.

Example:

#### Writer
```cpp
#include <tyndall/ipc/ipc.h>

my_struct entry = { .field = 42 };

ipc_write(entry, "my_topic");
```

#### Reader
```cpp
#include <tyndall/ipc/ipc.h>

while (1)
{
  my_struct entry;

  if (ipc_read(entry, "my_topic") == 0)
    printf("entry field: %d\n", entry.field);
}
```
Full example in [examples/ipc/](examples/ipc/).

### Read from command line

The [ipc\_read](tools/ipc_read.cpp) tool can be used to print a topic, using the name or path:

```shell
tyndall_tool_ipc_read ipc1ef42bc4e0bbfeb0ac34bc3642732768cf6f77b7_my_topic_891174619
tyndall_tool_ipc_read /dev/shm/ipc1ef42bc4e0bbfeb0ac34bc3642732768cf6f77b7_my_topic_891174619
```

For [aggregate initializable](https://en.cppreference.com/w/cpp/language/aggregate_initialization) entry types, this will attempt to print the fields of the entry.
If the entry is not aggregate initializable, the hexadecimal representation of the entry will be printed.
You can still get a formatted print of non-aggregate-initializables by supplying the format yourself:

```shell
tyndall_tool_ipc_read ipc1ef42bc4e0bbfeb0ac34bc3642732768cf6f77b7_my_topic_891174619 f
```

The format is a list of type ids according to:
```cpp
s = string (null terminated)
b = bool
f = float
d = double
i = int
j = unsigned int
c = char
h = unsigned char
l = long
m = unsigned long
x = long long
y = unsigned long long
```

So to print a struct `struct S{ int a; unsigned char b; float c, d; S(){}}` you would run:
```shell
tyndall_tool_ipc_read ipc1ef42bc4e0bbfeb0ac34bc3642732768cf6f77b7_my_topic_891174619 ihff
```

Note that the entry is assumed to have standard alignment.

You can ignore a specified amount of bytes in the struct by having a number in the format:
```shell
tyndall_tool_ipc_read ipc1ef42bc4e0bbfeb0ac34bc3642732768cf6f77b7_my_topic_891174619 i8f
```
