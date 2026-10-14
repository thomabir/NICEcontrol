// Check of nF_EBD_Controller against live nanoFaktur controllers on the LAN.
// Usage: nf_controller <serial number> [<serial number> ...]
// The stages move between 2 mrad and 4 mrad, and stop at 3 mrad.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "devices/nF_EBD_Controller.hpp"

namespace {

constexpr double tolerance = 0.01;  // mrad
constexpr auto settle_time = std::chrono::milliseconds(500);

int failures = 0;

void check(bool ok, const std::string &description) {
  std::printf("%s: %s\n", ok ? "PASS" : "FAIL", description.c_str());
  if (!ok) {
    failures++;
  }
}

bool near(std::array<double, 2> position, std::array<double, 2> target) {
  return std::abs(position[0] - target[0]) < tolerance && std::abs(position[1] - target[1]) < tolerance;
}

// Sends the target, then reads without pause, as a control loop does. The read must see the stage settle.
bool move_and_check(nF_EBD_Controller &controller, std::array<double, 2> target) {
  controller.move_to(target);
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < settle_time) {
    controller.read();
  }
  return near(controller.read(), target);
}

double median_read_ms(nF_EBD_Controller &controller, int n_reads) {
  std::vector<double> latency_ms;
  for (int i = 0; i < n_reads; i++) {
    const auto t0 = std::chrono::steady_clock::now();
    controller.read();
    latency_ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  }
  std::sort(latency_ms.begin(), latency_ms.end());
  return latency_ms[latency_ms.size() / 2];
}

void check_one(nF_EBD_Controller &controller, const std::string &serial) {
  check(controller.is_connected(), serial + ": connected");
  check(move_and_check(controller, {3.0, 3.5}), serial + ": move to (3.0, 3.5) with reads without pause");
  check(move_and_check(controller, {2.5, 2.0}), serial + ": move to (2.5, 2.0) with reads without pause");

  for (int i = 0; i <= 200; i++) {
    controller.move_to({2.0 + 0.01 * i, 4.0 - 0.01 * i});
  }
  std::this_thread::sleep_for(settle_time);
  check(near(controller.read(), {4.0, 2.0}), serial + ": the last of 200 fast targets is reached");

  std::printf("  %s: median read() time %.3f ms\n", serial.c_str(), median_read_ms(controller, 500));

  controller.move_to({3.0, 3.0});
  std::this_thread::sleep_for(settle_time);
  controller.close();
  check(!controller.is_connected(), serial + ": is_connected() is false after close()");
  check(controller.init(), serial + ": init() after close() connects again");
  check(near(controller.read(), {3.0, 3.0}), serial + ": position is kept after close() and init()");
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::printf("usage: %s <serial number> [<serial number> ...]\n", argv[0]);
    return 2;
  }

  nF_EBD_Controller unknown("0000000000");
  check(!unknown.init(), "init() fails for a serial number that is not on the LAN");

  std::vector<std::unique_ptr<nF_EBD_Controller>> controllers;
  for (int i = 1; i < argc; i++) {
    controllers.push_back(std::make_unique<nF_EBD_Controller>(argv[i]));
    check(controllers.back()->init(), std::string(argv[i]) + ": init()");
  }

  for (int i = 1; i < argc; i++) {
    if (controllers[i - 1]->is_connected()) {
      check_one(*controllers[i - 1], argv[i]);
    }
  }

  // All controllers at the same time, one thread each, as the apps of the control software do.
  std::vector<std::thread> threads;
  std::vector<char> ok(controllers.size(), 0);
  for (size_t i = 0; i < controllers.size(); i++) {
    threads.emplace_back([&, i] {
      const std::array<double, 2> target = {2500.0 + 200.0 * i, 3500.0 - 200.0 * i};
      ok[i] = controllers[i]->is_connected() && move_and_check(*controllers[i], target);
    });
  }
  for (std::thread &t : threads) {
    t.join();
  }
  check(std::all_of(ok.begin(), ok.end(), [](char c) { return c != 0; }), "all controllers in parallel threads");

  for (auto &c : controllers) {
    c->move_to({3.0, 3.0});
  }
  std::this_thread::sleep_for(settle_time);
  for (auto &c : controllers) {
    c->close();
  }

  std::printf("%d failures\n", failures);
  return failures == 0 ? 0 : 1;
}
