#pragma once

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "lib/nF/nF_interface.h"

namespace nF_EBD_detail {

constexpr int discovery_port = 51000;
constexpr const char *discovery_message = "nF";
constexpr int discovery_timeout_ms = 500;
constexpr u32 param_device_serial = 0xFF000011;
constexpr u8 closed_loop = 1;
constexpr int cmd_level = 0xFFF0;

struct DiscoveredController {
  std::string ip;
  std::string name;
  std::string serial_number;
  std::string status;
};

inline std::string trim(const std::string &s) {
  const size_t first = s.find_first_not_of(" \0", 0, 2);
  if (first == std::string::npos) {
    return "";
  }
  const size_t last = s.find_last_not_of(" \0", std::string::npos, 2);
  return s.substr(first, last - first + 1);
}

// Parses a discovery response of the form "<name>, SN:<serial number>, <status>".
inline bool parse_response(const std::string &response, DiscoveredController &controller) {
  const size_t comma_1 = response.find(',');
  if (comma_1 == std::string::npos) {
    return false;
  }
  const size_t sn = response.find("SN:", comma_1);
  if (sn == std::string::npos) {
    return false;
  }
  const size_t comma_2 = response.find(',', sn);
  controller.name = trim(response.substr(0, comma_1));
  controller.serial_number =
      trim(response.substr(sn + 3, comma_2 == std::string::npos ? std::string::npos : comma_2 - sn - 3));
  controller.status = comma_2 == std::string::npos ? "" : trim(response.substr(comma_2 + 1));
  return true;
}

// Sends the discovery message to the broadcast address of each IPv4 interface and collects the responses.
inline std::vector<DiscoveredController> discover(int timeout_ms) {
  std::vector<DiscoveredController> controllers;
  const int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    return controllers;
  }
  const int on = 1;
  setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));

  ifaddrs *interfaces = nullptr;
  if (getifaddrs(&interfaces) == 0) {
    for (ifaddrs *ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET || ifa->ifa_broadaddr == nullptr) {
        continue;
      }
      if (!(ifa->ifa_flags & IFF_UP) || !(ifa->ifa_flags & IFF_BROADCAST) || (ifa->ifa_flags & IFF_LOOPBACK)) {
        continue;
      }
      sockaddr_in destination = *reinterpret_cast<sockaddr_in *>(ifa->ifa_broadaddr);
      destination.sin_port = htons(discovery_port);
      sendto(sock, discovery_message, std::strlen(discovery_message), 0, reinterpret_cast<sockaddr *>(&destination),
             sizeof(destination));
    }
    freeifaddrs(interfaces);
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (true) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      break;
    }
    pollfd pfd = {sock, POLLIN, 0};
    if (poll(&pfd, 1, static_cast<int>(remaining.count())) <= 0) {
      break;
    }
    char buffer[512];
    sockaddr_in source{};
    socklen_t source_len = sizeof(source);
    const ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&source), &source_len);
    if (len <= 0) {
      continue;
    }
    DiscoveredController controller;
    if (!parse_response(std::string(buffer, static_cast<size_t>(len)), controller)) {
      continue;
    }
    char ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &source.sin_addr, ip, sizeof(ip));
    controller.ip = ip;
    const bool known = std::any_of(controllers.begin(), controllers.end(),
                                   [&](const DiscoveredController &c) { return c.ip == controller.ip; });
    if (!known) {
      controllers.push_back(controller);
    }
  }
  ::close(sock);
  return controllers;
}

// nF_intf_disconnect() does not return: it stops the calling thread like pthread_exit(). Thus it runs in a separate
// POSIX thread. A std::thread is not used, because a pthread_exit() in a std::thread can call std::terminate().
inline void *disconnect_thread(void *fd) {
  nF_intf_disconnect(*static_cast<int *>(fd));
  return nullptr;
}

}  // namespace nF_EBD_detail

// nanoFaktur EBD-120 tip/tilt piezo controller (2 axes), connected via LAN (TCP port 51000).
// The controller is found by its device serial number with a UDP broadcast on all IPv4 interfaces, so the IP address
// is not necessary. The controller must use DHCP (parameter 0xFF010001 = 1 in flash).
// Positions and targets are in urad.
class nF_EBD_Controller {
 public:
  explicit nF_EBD_Controller(const char *serial_number)
      : serial_number(serial_number), name(std::string("nF EBD Controller S/N ") + serial_number) {}

  ~nF_EBD_Controller() { close(); }

  nF_EBD_Controller(const nF_EBD_Controller &) = delete;
  nF_EBD_Controller &operator=(const nF_EBD_Controller &) = delete;

  // Finds and connects the controller, verifies its serial number, reads the closed-loop range and switches on closed
  // loop on both axes. Returns false if one of these steps fails.
  bool init() {
    if (this->fd >= 0) {
      return true;
    }
    if (!find_ip()) {
      return false;
    }

    this->fd = nF_intf_connect_tcpip(this->ip.data());
    if (this->fd < 0) {
      std::cout << this->name << ": Connection to " << this->ip << " failed (error " << this->fd << ")" << std::endl;
      this->fd = -1;
      return false;
    }

    if (!verify_serial_number() || !read_range() || !enable_servo()) {
      disconnect();
      return false;
    }

    {
      std::scoped_lock lock(this->target_mutex);
      this->stop_mover = false;
      this->target_pending = false;
    }
    this->mover = std::thread(&nF_EBD_Controller::move_loop, this);

    std::cout << this->name << ": Connection successful (" << this->ip << ")" << std::endl;
    return true;
  }

  bool is_connected() {
    std::scoped_lock lock(this->io_mutex);
    return this->fd >= 0;
  }

  // Position of both axes (urad). A read takes one TCP round trip (approximately 1.3 ms). NaN if not connected or if
  // the read fails. With -ffast-math (included in -Ofast), std::isnan() does not detect NaN, so use is_connected().
  std::array<double, 2> read() {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    std::scoped_lock lock(this->io_mutex);
    if (this->fd < 0) {
      return {nan, nan};
    }
    send_pending_target();
    float position[2];  // mrad
    if (nF_get_dev_axis_position_m(this->fd, 2, this->axes, position) != 0) {
      return {nan, nan};
    }
    return {position[0] * 1e3 - this->offset, position[1] * 1e3 - this->offset};
  }

  // Sets a closed-loop target (urad) and returns immediately. A worker thread or the next read() sends the target,
  // whichever gets the connection first. Thus a read() after move_to() always comes after the command. Targets that
  // arrive before the send replace each other, and the last target is always sent. Targets outside the closed-loop
  // range of the controller are clamped.
  void move_to(std::array<double, 2> target) {
    {
      std::scoped_lock lock(this->target_mutex);
      this->pending_target = target;
      this->target_pending = true;
    }
    this->target_cv.notify_one();
  }

  void close() {
    if (this->mover.joinable()) {
      {
        std::scoped_lock lock(this->target_mutex);
        this->stop_mover = true;
      }
      this->target_cv.notify_one();
      this->mover.join();
    }
    disconnect();
  }

 private:
  std::string serial_number;
  std::string name;
  std::string ip;
  int fd = -1;                      // interface descriptor of the vendor library, -1 if not connected
  int axes[2] = {0, 1};             // x-axis, y-axis
  double offset = 0.0;              // urad
  std::array<double, 2> min_pos{};  // urad, closed-loop range from the controller
  std::array<double, 2> max_pos{};  // urad
  std::mutex io_mutex;              // one command at a time on the connection

  std::thread mover;
  std::mutex target_mutex;
  std::condition_variable target_cv;
  std::array<double, 2> pending_target{};
  bool target_pending = false;
  bool stop_mover = false;

  bool find_ip() {
    int matches = 0;
    for (const nF_EBD_detail::DiscoveredController &controller :
         nF_EBD_detail::discover(nF_EBD_detail::discovery_timeout_ms)) {
      if (controller.serial_number == this->serial_number) {
        this->ip = controller.ip;
        matches++;
        if (controller.status.find("waiting for connection") == std::string::npos) {
          std::cout << this->name << ": Controller status: " << controller.status << std::endl;
        }
      }
    }
    if (matches == 0) {
      std::cout << this->name << ": Connection failed, no controller with this serial number on the LAN" << std::endl;
      return false;
    }
    if (matches > 1) {
      std::cout << this->name << ": Connection failed, more than one controller with this serial number" << std::endl;
      return false;
    }
    return true;
  }

  bool verify_serial_number() {
    u8 format = 0;
    ParamVal value{};
    const int err = nF_get_dev_parameter_ram_m(this->fd, 0, nF_EBD_detail::param_device_serial, &format, &value);
    value.sData[MAX_PARAMETER_STRLEN - 1] = '\0';
    if (err != 0 || format != CMD_DATA_FMT_STRING || this->serial_number != value.sData) {
      std::cout << this->name << ": Error: Controller at " << this->ip << " has a different serial number" << std::endl;
      return false;
    }
    return true;
  }

  bool read_range() {
    float low[2], high[2];  // mrad
    u32 level = 1;
    nF_intf_write_command_u32_m(this->fd, nF_EBD_detail::cmd_level, 1, &level);
    const int err_low = nF_get_dev_axis_range_low_m(this->fd, nF_EBD_detail::closed_loop, 2, this->axes, low);
    const int err_high = nF_get_dev_axis_range_high_m(this->fd, nF_EBD_detail::closed_loop, 2, this->axes, high);
    if (err_low != 0 || err_high != 0) {
      std::cout << this->name << ": Error: Cannot read the closed-loop range (errors " << err_low << ", " << err_high
                << ", controller error " << nF_get_dev_error_m(this->fd) << ")" << std::endl;
      return false;
    }
    for (int i = 0; i < 2; i++) {
      this->min_pos[i] = low[i] * 1e3 - this->offset;
      this->max_pos[i] = high[i] * 1e3 - this->offset;
    }
    return true;
  }

  // Sets the closed-loop target to the present position before closed loop starts, thus the axes do not move.
  bool enable_servo() {
    int servo[2];
    if (nF_get_dev_axis_svo_m(this->fd, 2, this->axes, servo) != 0) {
      std::cout << this->name << ": Error: Cannot read the servo status" << std::endl;
      return false;
    }
    if (servo[0] == 1 && servo[1] == 1) {
      return true;
    }

    float position[2];
    nF_get_dev_axis_position_m(this->fd, 2, this->axes, position);
    nF_set_dev_axis_target_m(this->fd, nF_EBD_detail::closed_loop, 2, this->axes, position);
    servo[0] = servo[1] = 1;
    nF_set_dev_axis_svo_m(this->fd, 2, this->axes, servo);

    nF_get_dev_axis_svo_m(this->fd, 2, this->axes, servo);
    for (int i = 0; i < 2; i++) {
      if (servo[i] != 1) {
        std::cout << this->name << ": Error: Servo mode not enabled for axis " << i << std::endl;
        return false;
      }
    }
    return true;
  }

  void move_loop() {
    while (true) {
      {
        std::unique_lock lock(this->target_mutex);
        this->target_cv.wait(lock, [this] { return this->target_pending || this->stop_mover; });
        if (this->stop_mover) {
          return;
        }
      }
      std::scoped_lock lock(this->io_mutex);
      send_pending_target();
    }
  }

  // Sends the pending target, if there is one. The caller holds io_mutex.
  void send_pending_target() {
    std::array<double, 2> target;
    {
      std::scoped_lock lock(this->target_mutex);
      if (!this->target_pending) {
        return;
      }
      target = this->pending_target;
      this->target_pending = false;
    }
    float target_mrad[2];  // the nF API expects float
    for (int i = 0; i < 2; i++) {
      const double clamped = std::clamp(target[i], this->min_pos[i], this->max_pos[i]);
      target_mrad[i] = static_cast<float>((clamped + this->offset) * 1e-3);
    }
    if (this->fd >= 0) {
      nF_set_dev_axis_target_m(this->fd, nF_EBD_detail::closed_loop, 2, this->axes, target_mrad);
    }
  }

  void disconnect() {
    std::scoped_lock lock(this->io_mutex);
    if (this->fd < 0) {
      return;
    }
    pthread_t thread;
    if (pthread_create(&thread, nullptr, nF_EBD_detail::disconnect_thread, &this->fd) == 0) {
      pthread_join(thread, nullptr);
    }
    this->fd = -1;
  }
};
