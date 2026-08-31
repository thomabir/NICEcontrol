#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <mutex>

// Sends file descriptor 1 to /dev/null while the object exists, then puts the previous target back.
// The USB layer of the PI GCS2 library dumps the full device descriptor to stdout and has no option to turn this off.
// The lock makes parallel connects safe against each other: without it, one thread can save a file descriptor that
// another thread already replaced, which makes /dev/null the permanent target of stdout.
// File descriptor 1 belongs to the whole process, so output of other threads is also lost in this window.
class ScopedStdoutSilence {
 public:
  ScopedStdoutSilence() : lock(mutex()) {
    flush();
    saved_stdout = dup(STDOUT_FILENO);
    null_target = open("/dev/null", O_WRONLY);
    if (saved_stdout >= 0 && null_target >= 0) {
      dup2(null_target, STDOUT_FILENO);
    }
  }

  ~ScopedStdoutSilence() {
    flush();
    if (saved_stdout >= 0) {
      dup2(saved_stdout, STDOUT_FILENO);
      ::close(saved_stdout);
    }
    if (null_target >= 0) {
      ::close(null_target);
    }
  }

  ScopedStdoutSilence(const ScopedStdoutSilence &) = delete;
  ScopedStdoutSilence &operator=(const ScopedStdoutSilence &) = delete;

 private:
  static void flush() {
    std::cout << std::flush;
    std::fflush(stdout);
  }

  static std::mutex &mutex() {
    static std::mutex shared_mutex;
    return shared_mutex;
  }

  std::scoped_lock<std::mutex> lock;
  int saved_stdout = -1;
  int null_target = -1;
};
