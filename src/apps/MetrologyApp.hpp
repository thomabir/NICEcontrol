#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include "core/Whiteboard.hpp"

// Reads the metrology ADC stream, which arrives over UDP. The socket does not block, so the core drains whatever has
// arrived and moves on.
class MetrologyApp {
 public:
  explicit MetrologyApp(Whiteboard &whiteboard) : wb(whiteboard) { open_socket(); }

  ~MetrologyApp() {
    if (sockfd >= 0) {
      ::close(sockfd);
    }
  }

  void sense() {
    if (sockfd < 0) {
      return;
    }

    char buffer[kPacketBytes];
    for (int packet = 0; packet < kMaxPacketsPerCycle; packet++) {
      const ssize_t bytes = ::recv(sockfd, buffer, sizeof(buffer), MSG_DONTWAIT);
      if (bytes < 0) {
        return;  // nothing left to read
      }
      if (bytes < static_cast<ssize_t>(kPacketBytes)) {
        wb.state.metrology.dropped_packets++;
        continue;
      }

      int words[kChannelsPerRecord * kTimepointsPerPacket];
      std::memcpy(words, buffer, sizeof(words));

      for (int i = 0; i < kTimepointsPerPacket; i++) {
        const int *record = &words[kChannelsPerRecord * i];
        AdcSample sample;
        sample.counter = record[0];
        for (int channel = 0; channel < 16; channel++) {
          // Channel 4 carries the sum of the four shear signals instead of its own input.
          sample.value[channel] =
              (channel == 4) ? (record[1] + record[2] + record[3] + record[4]) : record[channel + 1];
        }

        wb.adc.push(sample);
        wb.state.metrology.counter = sample.counter;
        wb.state.metrology.sample_count++;
      }
    }
  }

 private:
  static constexpr int kPortNumber = 12345;
  static constexpr int kChannelsPerRecord = 17;  // one counter and sixteen channels
  static constexpr int kTimepointsPerPacket = 10;
  static constexpr size_t kPacketBytes = sizeof(int) * kChannelsPerRecord * kTimepointsPerPacket;
  static constexpr int kMaxPacketsPerCycle = 200;

  Whiteboard &wb;
  int sockfd = -1;

  void open_socket() {
    sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
      std::cerr << "MetrologyApp: cannot create the socket." << std::endl;
      return;
    }

    struct sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(kPortNumber);
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(sockfd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
      std::cerr << "MetrologyApp: cannot bind the socket to port " << kPortNumber << "." << std::endl;
      ::close(sockfd);
      sockfd = -1;
      return;
    }

    wb.state.metrology.socket_open = true;
  }
};
