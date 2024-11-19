#include <errno.h>
#include <fcntl.h>
#include <google/protobuf/util/delimited_message_util.h>

#include "tyndall/meta/strval.h"

int binlog_create(const char *path) {
  return open(path, O_CREAT | O_WRONLY, S_IRUSR | S_IWUSR);
}

template <typename ProtoMessage>
  requires(std::is_base_of<::google::protobuf::Message, ProtoMessage>::value)
int binlog_write(int fd, const ProtoMessage &proto) {
  if (fd == -1) {
    errno = ENOENT;
    return -1;
  } else {
    if (::google::protobuf::util::SerializeDelimitedToFileDescriptor(proto, fd))
      return 0;
    else {
      errno = EPROTO;
      return -1;
    }
  }
}

int binlog_open(const char *path) { return open(path, O_RDONLY); }

template <typename ProtoMessage>
  requires(std::is_base_of<::google::protobuf::Message, ProtoMessage>::value)
int binlog_read(int fd, ProtoMessage &proto) {
  if (fd == -1) {
    errno = ENOENT;
    return -1;
  } else {
    static ::google::protobuf::io::FileInputStream *stream = NULL;
    static int fd_save = -1;
    if (fd != fd_save) {
      fd_save = fd;
      delete stream;
      stream = new ::google::protobuf::io::FileInputStream(fd);
    }

    bool clean_eof;
    if (::google::protobuf::util::ParseDelimitedFromZeroCopyStream(
            &proto, stream, &clean_eof))
      return 0;
    else {
      delete stream;

      if (clean_eof)
        errno = EIO;
      else
        errno = stream->GetErrno();
      return -1;
    }
  }
}
