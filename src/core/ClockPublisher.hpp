#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

#include "client/nice_clock.h"

/**
 * @brief Writes the two clocks into a file in shared memory, for every other program on this PC.
 *
 * client/nice_clock.h holds the record and the reader for those programs.
 * The caller writes a record only while the offset of the two clocks is good, thus a fresh record is a good record.
 * A file lock guards the record, and the kernel releases the lock of a process that dies.
 * A failure prints one line, and the program runs on without the record.
 */
class ClockPublisher {
 public:
  /** @param path the record, NICE_CLOCK_PATH by default. */
  explicit ClockPublisher(const char *path = NICE_CLOCK_PATH) : path_(path) {
    fd_ = ::open(path, O_CREAT | O_RDWR, 0644);
    if (fd_ == -1) {
      std::cerr << "ClockPublisher: " << path_ << " does not open (" << std::strerror(errno)
                << "). The other programs on this PC do not get the two clocks." << std::endl;
      return;
    }
    fchmod(fd_, 0644);  // an earlier umask can have left another mode on the file
  }

  ~ClockPublisher() {
    if (fd_ != -1) {
      ::close(fd_);
    }
  }

  ClockPublisher(const ClockPublisher &) = delete;
  ClockPublisher &operator=(const ClockPublisher &) = delete;

  /**
   * @brief Writes one record over the record of the last cycle.
   * @param mono_to_PC_ns        t_PC = CLOCK_MONOTONIC + mono_to_PC_ns
   * @param mono_to_DC_ns        t_DC = CLOCK_MONOTONIC + mono_to_DC_ns
   * @param published_at_mono_ns the CLOCK_MONOTONIC of this record
   */
  void publish(int64_t mono_to_PC_ns, int64_t mono_to_DC_ns, int64_t published_at_mono_ns) {
    const NiceClockRecord record = {mono_to_PC_ns, mono_to_DC_ns, published_at_mono_ns};
    if (fd_ == -1 || nice_clock_flock(fd_, LOCK_EX) == -1) {
      return;
    }
    // The record of the last cycle stays after a failure, it gets old, and the reader then refuses it.
    if (pwrite(fd_, &record, sizeof(record), 0) < 0 && !write_failed_) {
      write_failed_ = true;
      std::cerr << "ClockPublisher: " << path_ << " does not take the record (" << std::strerror(errno)
                << "). This line comes one time only." << std::endl;
    }
    nice_clock_flock(fd_, LOCK_UN);
  }

 private:
  std::string path_;
  int fd_ = -1;
  bool write_failed_ = false;
};
