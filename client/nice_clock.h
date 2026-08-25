#ifndef NICE_CLOCK_H
#define NICE_CLOCK_H

/**
 * @file nice_clock.h
 * @brief Facilities so that the current EtherCAT Distributed Clock (DC) time can be derived.
 *
 * NICEcontrol regularly (100 Hz) reads the DC clock via a PCIe Ethercat Subdevice, and publishes the offset between it and the PC's local monotonic clock to a record in shared memory at 100 Hz.
 *
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <time.h>
#include <unistd.h>

/** Path of clock record */
#define NICE_CLOCK_PATH "/dev/shm/nice_clock"

/** @brief NICEcontrol's clock record, for reconstructing t_DC from the PC's local clock.
 *
 * t_PC and t_DC count ns from 2000-01-01 00:00.
 * Records are only written when the clock is deemed good by NICEcontrol.
 */
struct NiceClockRecord {
  /** The offset to derive a sort-of monotonic system clock: CLOCK_MONOTONIC but with an offset calculated at NICEcontrol startup so it counts ns from 2000-01-01, rather than from boot. t_PC = CLOCK_MONOTONIC + mono_to_PC_ns */
  int64_t mono_to_PC_ns;

  /** The offset to derive the EtherCAT DC time. It is the clock of the EtherCAT maindevice, and it will drift from UTC. t_DC = CLOCK_MONOTONIC + mono_to_DC_ns */
  int64_t mono_to_DC_ns;

  /** The CLOCK_MONOTONIC of this record. */
  int64_t published_at_mono_ns;
};

/**
 * @brief Takes or releases the lock of the record, and waits again after a signal.
 * @param fd        the open record
 * @param operation LOCK_SH, LOCK_EX, or LOCK_UN
 * @return 0, or -1 with errno
 */
static inline int nice_clock_flock(int fd, int operation) {
  int result;
  do {
    result = flock(fd, operation);
  } while (result == -1 && errno == EINTR);
  return result;
}

/**
 * @brief Calculates the present t_DC and t_PC from a record and the present CLOCK_MONOTONIC.
 *
 * @param record  The NiceClockRecord
 * @param mono_ns the reading of CLOCK_MONOTONIC, in ns
 * @param t_DC_ns Derived time of the EtherCAT DC clock at mono_ns
 * @param t_PC_ns NULL, or the derived internal clock of NICEcontrol at mono_ns
 * @param age_ns  NULL, or the age of the record at mono_ns
 */
static inline void nice_clock_from_record(const struct NiceClockRecord *record, int64_t mono_ns, int64_t *t_DC_ns,
                                          int64_t *t_PC_ns, int64_t *age_ns) {
  *t_DC_ns = mono_ns + record->mono_to_DC_ns;
  if (t_PC_ns != NULL) *t_PC_ns = mono_ns + record->mono_to_PC_ns;
  if (age_ns != NULL) *age_ns = mono_ns - record->published_at_mono_ns;
}

/**
 * @brief The present timestamps (t_PC and t_DC), derived from the most recent clock record.
 *
 * The record can be of any age, thus the caller judges age_ns against its own budget. NICEcontrol writes each 10 ms, and the record carries no rate, thus the error grows by about 50 us in each second of age.
 * Each failure prints one line to stderr, thus a caller that asks many times each second must look at the result and not at the log.
 *
 * @param t_DC_ns    Time of the EtherCAT DC clock, ns from 2000-01-01 00:00
 * @param t_PC_ns    NULL, or the derived internal clock of NICEcontrol in the same epoch, which is not UTC
 * @param age_ns     NULL, or the age of the record
 * @return 0, or -1 if the record does not open or does not read
 */
static inline int nice_clock_now(int64_t *t_DC_ns, int64_t *t_PC_ns, int64_t *age_ns) {
  struct NiceClockRecord record;
  struct timespec mono;
  int64_t mono_ns;
  int got = 0;
  int fd;

  const char *path = NICE_CLOCK_PATH;

  // open clock record
  fd = open(path, O_RDONLY);
  if (fd == -1) {
    fprintf(stderr, "No DC clock available: %s does not open (%s).\n", path, strerror(errno));
    return -1;
  }
  if (nice_clock_flock(fd, LOCK_SH) == 0) {
    // read clock record and get CLOCK_MONOTONIC
    got =
        pread(fd, &record, sizeof(record), 0) == (ssize_t)sizeof(record) && clock_gettime(CLOCK_MONOTONIC, &mono) == 0;
    nice_clock_flock(fd, LOCK_UN);
  }
  close(fd);

  // check if there is data in the clock record
  if (!got) {
    fprintf(stderr, "No DC clock available: %s does not give a record (%s).\n", path, strerror(errno));
    return -1;
  }

  mono_ns = (int64_t)mono.tv_sec * 1000000000LL + (int64_t)mono.tv_nsec;
  nice_clock_from_record(&record, mono_ns, t_DC_ns, t_PC_ns, age_ns);
  return 0;
}

#endif /* NICE_CLOCK_H */
