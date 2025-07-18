#pragma once

// #include <arpa/inet.h>   // ethernet
// #include <netinet/in.h>  // ethernet
// #include <sys/socket.h>  // ethernet

// #include <atomic>
// #include <cmath>
// #include <cstring>
// #include <iostream>
// #include <numbers>

#include <thread>

#include "EthercatResources.hpp"
// #include "utils.hpp"

#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <bitset>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#define ETH_P_ALL 0x0003             // Capture every packet
#define ETH_P_ECAT 0x88A4            // EtherCAT EtherType
const char* INTERFACE = "enp6s0f0";  // Ethernet interface to listen on, find with `ifconfig`
const uint8_t VALID_SOURCE_MAC[6] = {0x03, 0x01, 0x01, 0x01, 0x01, 0x01};

// configuration for slave offsets in buffer
const int HEADER_SIZE = 26;  // Size of the EtherCAT header in bytes
const int TOTAL_SLAVES = 3;

// SLAVE #1 - Gaetway
const int SLAVE_1_INPUT_BYTES = 64;  // "Input" = from MASTER to SLAVE
const int SLAVE_1_OUTPUT_BYTES = 64;

// SLAVE #2 - Delay Line
const int SLAVE_2_INPUT_BYTES = 5;
const int SLAVE_2_OUTPUT_BYTES = 4;

// SLAVE #3 - Metrology
const int SLAVE_3_INPUT_BYTES = 32;
const int SLAVE_3_OUTPUT_BYTES = 32;

// Vector to hold the number of bytes for each slave's input and output
// Note: The frame first contains all the input bytes of all slaves, followed by all the output bytes of all slaves.
const std::vector<int> SLAVE_BYTES = {SLAVE_1_INPUT_BYTES,  SLAVE_2_INPUT_BYTES,  SLAVE_3_INPUT_BYTES,
                                      SLAVE_1_OUTPUT_BYTES, SLAVE_2_OUTPUT_BYTES, SLAVE_3_OUTPUT_BYTES};

float get_float(const uint8_t* data) {
  float value;
  std::memcpy(&value, data, sizeof(float));
  return value;
}

int get_int(const uint8_t* data) {
  int value;
  std::memcpy(&value, data, sizeof(int));
  return value;
}

int64_t get_int64_t(const uint8_t* data) {
  int64_t value;
  std::memcpy(&value, data, sizeof(int64_t));
  return value;
}

float signed16BitToFloat(int value) { return value * M_PI / (32767); }

float opdRadToDLNm(float value) {
  return value * 1550 / (2 * M_PI);  // Convert radians from OPD measurement to nm
}

class EthercatTapReader {
 public:
  // constructor: take reference to metrology resources as input
  EthercatTapReader(EthercatResources& ecat_res) : ecat_res(ecat_res) {
    setup();                                   // setup the socket
    locations = get_slave_packet_locations();  // get the slave packet locations
  }

  // // Destructor to clean up resources
  // ~EthercatTapReader() {
  //   request_stop();
  //   // if (sockfd >= 0) {
  //   //   close(sockfd);
  //   // }
  // }

  // Start the thread
  void start() {
    if (calculation_thread.joinable()) {
      return;
    }  // already running
    calculation_thread = std::jthread([this](std::stop_token st) {
      std::cout << "Stop token state: " << st.stop_requested() << std::endl;
      while (!st.stop_requested()) {
        process_ethercat_packet();
      }
    });
    // if (!calculation_thread.joinable()) {
    //   running.store(true);
    //   calculation_thread = std::jthread([this](std::stop_token st) {
    //     while (!st.stop_requested()) {
    //       process_ethercat_packet();
    //     }
    //   });
    // }
  }

  // Stop the thread
  void request_stop() {
    if (calculation_thread.joinable()) {
      calculation_thread.request_stop();
    }
  }

 private:
  EthercatResources& ecat_res;
  int sock = -1;  // socket for UDP
  std::jthread calculation_thread;
  std::vector<int> locations;  // vector to hold slave packet locations

  std::vector<int> get_slave_packet_locations() {
    std::vector<int> locations;
    int offset = HEADER_SIZE;                      // Start after the EtherCAT header
    int total_start_positions = 2 * TOTAL_SLAVES;  // 2 for each slave (input and output)

    for (int i = 0; i < total_start_positions; ++i) {
      locations.push_back(offset);
      offset += SLAVE_BYTES[i];
    }
    return locations;
  }

  int setup() {
    // Create a raw socket to capture all packets
    sock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sock < 0) {
      std::cerr << "Failed to create socket." << std::endl;
      return 1;
    }

    // Set socket receive timeout to 100 ms
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100 ms
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
      std::cerr << "Failed to set socket receive timeout." << std::endl;
      close(sock);
      return 1;
    }

    sockaddr_ll sll = {};
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex = if_nametoindex(INTERFACE);

    if (bind(sock, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
      std::cerr << "Failed to bind socket." << std::endl;
      close(sock);
      return 1;
    }

    return 0;
  }

  EthercatData parse_frame(const uint8_t* packet, size_t length) {
    size_t process_data_length = length - 28;

    if (process_data_length < 170) {
      std::cerr << "Process data too short to decode all fields." << std::endl;
      return EthercatData();  // Return an empty EthercatData object
    }

    int buffer_index = get_int(packet + locations[0]);
    int64_t timestamp_us = get_int64_t(packet + locations[0] + 12);

    // delay line
    float dl_position_cmd = get_float(packet + locations[1]);
    float dl_position_meas = get_float(packet + locations[TOTAL_SLAVES + 1]);

    // metrology
    float metr_opd_nm_unwrapped = get_float(packet + locations[2] + 4);
    int metr_opd_int = get_int(packet + locations[TOTAL_SLAVES + 2]);

    // int working_counter = get_int(packet + length - 2); // Last 2 bytes of the packet

    // process
    float metr_opd_rad_wrapped = -signed16BitToFloat(metr_opd_int);

    static EthercatData data;
    data.buffer_index = buffer_index;
    data.timestamp_us = timestamp_us;
    data.dl_position_cmd = dl_position_cmd;
    data.dl_position_meas = dl_position_meas;
    data.metr_opd_rad_wrapped = metr_opd_rad_wrapped;
    data.metr_opd_nm_unwrapped = metr_opd_nm_unwrapped;

    return data;
  }

  // Wait for data, parse it, put in the EthercatResources data buffer
  void process_ethercat_packet() {
    static uint8_t buffer[65535];

    ssize_t length = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);  // length of the received packet
    if (length < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // std::cerr << "No data received within timeout." << std::endl;
        return;
      }
      std::cerr << "Failed to receive packet." << std::endl;
      return;
    }

    uint16_t eth_type = ntohs(*reinterpret_cast<uint16_t*>(buffer + 12));  // get EtherType from the packet
    if (eth_type != ETH_P_ECAT) {
      std::cerr << "Received packet is not EtherCAT." << std::endl;
      return;
    }

    // only read one way of the traffic, skip the other direction
    const uint8_t* src_mac = buffer + 6;
    if (std::memcmp(src_mac, VALID_SOURCE_MAC, 6) != 0) {
      // std::cerr << "Received packet from wrong source MAC address." << std::endl;
      return;
    }

    EthercatData data = parse_frame(buffer, length);

    // print data for debugging
    // std::cout << "t=" << data.timestamp_us << " DL: cmd=" << data.dl_position_cmd << ", meas=" <<
    // data.dl_position_meas
    //           << " | Metr: OPD rad wrapped=" << data.metr_opd_rad_wrapped
    //           << ", OPD nm unwrapped=" << data.metr_opd_nm_unwrapped << std::endl;

    // put data into the EthercatResources data buffer
    ecat_res.data.push(data);
  }
};
