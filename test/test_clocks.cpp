// Checks the two clocks in src/core/Clocks.hpp, the record that src/core/ClockPublisher.hpp writes, and the sums that client/nice_clock.h makes from that record.
// The publisher writes the record of this test, thus the record of a running NICEcontrol stays as it is.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "core/ClockPublisher.hpp"
#include "core/Clocks.hpp"

static int failures = 0;

static void expect(const std::string &what, bool ok) {
  std::printf("%-52s %s\n", what.c_str(), ok ? "OK" : "FAIL");
  if (!ok) failures++;
}

// A time of the PC clock in the year 2025, far from the epoch of the two clocks.
static constexpr int64_t kNow = 812345678901234567LL;
static constexpr const char *kPath = "/dev/shm/nice_clock_test";

int main() {
  Clocks clocks;
  const Timestamp none = clocks.stamp_from_t_PC(kNow);
  expect("no estimate: t_DC is a copy of t_PC", none.t_DC == kNow && !none.dc_good);

  // The offset holds at the time of the estimate, and the rate of 1 ppm moves it after that.
  clocks.set_offset(1000, 1e-6, kNow);
  const Timestamp good = clocks.stamp_from_t_PC(kNow + 50'000'000);
  expect("the rate moves the offset", good.t_DC == kNow + 50'000'000 + 1000 + 50);
  expect("a young estimate is good", good.dc_good);
  expect("the estimate is good at the age limit", clocks.stamp_from_t_PC(kNow + Clocks::kGoodAgeNs).dc_good);
  expect("and it is not good after that", !clocks.stamp_from_t_PC(kNow + Clocks::kGoodAgeNs + 1).dc_good);

  const Timestamp now = clocks.stamp_now();
  expect("a time of the bus and back", std::llabs(clocks.stamp_from_t_DC(now.t_DC).t_PC - now.t_PC) < 1'000'000);

  // The record for the other programs on this PC.
  {
    ClockPublisher publisher(kPath);
    clocks.set_offset(1000, 0.0, clocks.t_PC_now());
    const int64_t mono_ns = clocks.t_PC_now() - clocks.mono_to_PC();
    publisher.publish(clocks.mono_to_PC(), clocks.mono_to_PC() + 1000, mono_ns);

    NiceClockRecord record = {};
    const int fd = open(kPath, O_RDONLY);
    expect("the record opens", fd != -1);
    expect("the record holds the three numbers", pread(fd, &record, sizeof(record), 0) == (ssize_t)sizeof(record));
    close(fd);

    int64_t t_DC_ns = 0;
    int64_t t_PC_ns = 0;
    int64_t age_ns = 0;
    nice_clock_from_record(&record, mono_ns, &t_DC_ns, &t_PC_ns, &age_ns);
    expect("the record gives the time of the PC", std::llabs(t_PC_ns - clocks.t_PC_now()) < 1'000'000);
    expect("the record gives the time of the bus", std::llabs(t_DC_ns - clocks.stamp_now().t_DC) < 1'000'000);
    expect("a record of this instant has no age", age_ns == 0);

    // One second later, the two times move with the reading of the monotonic clock and the age grows by that second.
    nice_clock_from_record(&record, mono_ns + 1'000'000'000, &t_DC_ns, &t_PC_ns, &age_ns);
    expect("the times follow the monotonic clock", t_DC_ns - t_PC_ns == 1000 && age_ns == 1'000'000'000);
  }
  std::remove(kPath);

  std::printf("%s\n", failures == 0 ? "all checks passed" : "checks failed");
  return failures == 0 ? 0 : 1;
}
