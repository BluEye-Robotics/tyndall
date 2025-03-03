#include <assert.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <typeinfo>

#include <algorithm>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <fcntl.h>
#include <fmt/format.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <tyndall/ipc/seq_lock.h>
#include <tyndall/reflect/print_format.h>
#include <unistd.h>
#include <vector>

void print(char *fmt, const char *buf, size_t buf_size) {
  const char *b = buf;
  int i = 0;
  for (char *f = fmt; (*f != '\0'); ++f) {
    if (b - buf > buf_size) {
      break;
    }
    switch (*f) {
    case 'c':
      if (*b == '\0') {
        // std::cout << i << "(" << *f << "):  skipping" << std::endl;
        b += sizeof(char) + print_format_alignment<char>(b);
        continue;
      }

      std::cout << i << "(" << *f << strlen(b) << "): ";
      while (*b != '\0') {
        size_t printed = print_format(*f, b);
        b += printed;
        f++;
      }
      b += sizeof(char) + print_format_alignment<char>(b);
      std::cout << std::endl;

      break;
    default:
      std::cout << i << "(" << *f << "): ";
      size_t printed = print_format(*f, b);
      if (printed == 0) {
        switch (*f) {
        case 's':
          printf("%ss, ", b);
          printed = strlen(b);
          b += printed;
          break;
        default:
          printf("error, wrong parameter: %c\n", *f);
        }
        if (printed == 0)
          break;
      } else {
        std::cout << std::endl;
        b += printed;
      }

      break;
    }
    i++;
  }
  std::cout << std::endl;
}

int main(int argc, char **argv) {
  namespace po = boost::program_options;

  std::string file;
  boost::optional<std::string> format;

  // Declare the supported options.
  po::options_description desc("Allowed options");

  auto opt = desc.add_options();
  opt("help,h", "Produce help message");
  opt("file", po::value(&file)->required(), "File to read");
  opt("format", po::value(&format),
      "Format to print. Pass in a string of "
      "characters to print the data in that format. Supported characters are:\n"
      " - b(bool)\n"
      " - f(float)\n"
      " - d(double)\n"
      " - i(int)\n"
      " - j(unsinged int)\n"
      " - s(short)\n"
      " - t(unsigned short)\n"
      " - c(char)\n"
      " - h(unsigned char)\n"
      " - l(long)\n"
      " - m(unsigned long)\n"
      " - x(long long)\n"
      " - y(unsigned long long)");

  // Declare which options are positional
  po::positional_options_description p;
  p.add("file", 1);
  p.add("format", 2);

  po::variables_map vm;
  try {
    po::store(
        po::command_line_parser(argc, argv).options(desc).positional(p).run(),
        vm);
    po::notify(vm);
  } catch (boost::program_options::unknown_option &ex) {
    std::cout << desc << std::endl;
    std::cerr << boost::diagnostic_information(ex);
    std::cout << desc << std::endl;
    return EXIT_FAILURE;
  } catch (boost::program_options::multiple_occurrences &ex) {
    std::cout << desc << std::endl;
    std::cerr << boost::diagnostic_information(ex);
  } catch (boost::wrapexcept<boost::program_options::required_option> &ex) {
    std::cerr << "Error: " << ex.what() << std::endl << std::endl;
    std::cout << desc << std::endl;
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return EXIT_SUCCESS;
  }

  const auto ipc_prefix = "/dev/shm/";

  if (file.starts_with(ipc_prefix)) {
    // Remove /dev/shm/ prefix
    file = file.substr(strlen(ipc_prefix));
  }

  // ipc
  int fd = shm_open(file.c_str(), O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
  assert(fd != -1);

  size_t ipc_size;
  {
    struct stat st;
    int rc = fstat(fd, &st);
    assert(rc == 0);
    ipc_size = st.st_size;
  }

  void *const mapped =
      mmap(NULL, ipc_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  assert(mapped != MAP_FAILED);
  // shared memory should be page aligned:
  assert(reinterpret_cast<uintptr_t>(mapped) % CACHELINE_BYTES == 0);

  // Seq lock number address
  unsigned *ipc_seq = (unsigned *)mapped;

  // Data size address
  size_t *data_size = (size_t *)(mapped + sizeof(unsigned));
  // Data address
  const char *const ipc_buf = (const char *)mapped + CACHELINE_BYTES;

  const size_t buf_size = ipc_size - 2 * sizeof(int) - CACHELINE_BYTES;

  std::vector<char> buffer(buf_size + CACHELINE_BYTES, 0);

  char *buf = buffer.data();

  // align buf
  {
    constexpr size_t alignment = CACHELINE_BYTES;
    size_t alignment_error = reinterpret_cast<uintptr_t>(buf) % alignment;
    if (alignment_error > 0)
      buf += alignment - alignment_error;
  }

  unsigned seq = 0;
  while (1) {
    bool new_buf = false;

    {
      unsigned seq1;
      do {
        seq1 = seq_lock_read_begin(ipc_seq);
        memcpy(buf, ipc_buf, buf_size);

      } while (seq_lock_read_retry(ipc_seq, seq1));

      if (seq1 != seq) {
        new_buf = true;
        seq = seq1;
      }
    }

    if (!new_buf) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    if (format.has_value()) {
      print(format.get().data(), buf, buf_size);
      continue;
    }

    // extract debug format
    constexpr int type_info_hash_size = 4;
    const size_t debug_tail_size = ipc_size - *data_size;
    const size_t debug_format_size = debug_tail_size - type_info_hash_size;

    char *const debug_format_loc = static_cast<char *>(mapped) + ipc_size -
                                   debug_format_size + CACHELINE_BYTES + 24;

    char *fmt = debug_format_loc;
    std::string fmt_string(fmt);

    std::vector<char> allowed = {'b', 'f', 'd', 'i', 'j', 's', 't',
                                 'c', 'h', 'l', 'm', 'x', 'y'};

    auto valid = std::all_of(fmt_string.begin(), fmt_string.end(), [&](char c) {
      return std::find(allowed.begin(), allowed.end(), c) != allowed.end();
    });

    if (debug_tail_size > type_info_hash_size && fmt_string.size() > 0 &&
        valid) {
      print(fmt, buf, buf_size);
    } else {
      auto len = *data_size;
      for (size_t i = 0; i < len; ++i)
        printf("%02x", buf[i]);
      printf("\n");
      // printf("int: %d\n", *(int*)buf);
    }
  }

  {
    int rc = munmap(mapped, ipc_size);
    assert(rc == 0);
  }

  return 0;
}
