#include <arpa/inet.h>   // ethernet
#include <netinet/in.h>  // ethernet
#include <sys/socket.h>  // ethernet

#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>

#include "SharedResources.hpp"

#pragma once

class MetrologyReader {
 public:
  // constructor: take reference to metrology resources as input
  MetrologyReader(MetrologyResources &met_res) : met_res(met_res) { sockfd = setup_ethernet(); }

  // Destructor to clean up resources
  ~MetrologyReader() {
    request_stop();
    if (sockfd >= 0) {
      close(sockfd);
    }
  }

  // Start the measurement thread
  void start() {
    if (!calculation_thread.joinable()) {
      running.store(true);
      calculation_thread = std::jthread(&MetrologyReader::run_calculation, this);
    }
  }

  // Stop the measurement thread
  void request_stop() {
    if (calculation_thread.joinable()) {
      running.store(false);
      calculation_thread.request_stop();
    }
  }

 private:
  MetrologyResources &met_res;
  int sockfd = -1;  // socket for UDP
  std::jthread calculation_thread;
  std::atomic<bool> running{false};  // atomic flag to control calculation thread

  // Setup ethernet connection
  int setup_ethernet() {
    // Create a UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
      std::cerr << "Failed to create socket." << std::endl;
    }

    // Set up the server address
    struct sockaddr_in serverAddr;
    std::memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    int PortNumber = 12345;
    serverAddr.sin_port = htons(PortNumber);         // Replace with the desired port number
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);  // Listen on all network interfaces

    // Bind the socket to the server address
    if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
      std::cerr << "Failed to bind socket." << std::endl;
    }

    return sockfd;
  }

  // Main calculation loop
  void run_calculation() {
    // int prev_counter = 0;
    int buffer_size = 1024;
    char buffer[buffer_size];

    while (running.load()) {
      // float shear_x1_f = 0., shear_x2_f = 0., shear_y1_f = 0., shear_y2_f = 0., point_x1_f = 0., point_x2_f = 0.,
      //       point_y1_f = 0., point_y2_f = 0., sci_null = 0.;
      // auto t = utils::getTime();

      // Receive data
      struct sockaddr_in clientAddr;
      socklen_t clientAddrLen = sizeof(clientAddr);
      int numBytes = recvfrom(sockfd, buffer, buffer_size, 0, (struct sockaddr *)&clientAddr, &clientAddrLen);

      // Check for errors
      if (numBytes < 0) {
        std::cerr << "Error receiving data." << std::endl;
        break;
      }

      if (!running.load()) break;

      // Convert received data to vector of ints
      const static int num_ch = 17;  // num of signals in received data
      const static int num_tp = 10;  // num of timepoints in received data
      int recvd_data[num_ch * num_tp];

      std::memcpy(recvd_data, buffer, sizeof(int) * num_ch * num_tp);

      // Process data for each timepoint
      static int32_t counter;
      for (int i = 0; i < num_tp; i++) {
        counter = recvd_data[num_ch * i];

        for (int j = 0; j < num_ch - 1; j++) {
          if (j != 7) {
            met_res.adc_queues[j].push({counter, recvd_data[num_ch * i + j + 1]});
          } else {
            // sum of all shear signals in adc channel 7
            int shear_sum = 0;
            for (int k = 1; k <= 4; k++) {
              shear_sum += recvd_data[num_ch * i + k + 1];
            }
            met_res.adc_queues[j].push({counter, shear_sum});
          }
        }
      }

      // Check for gaps within the package
      // for (int i = 1; i < num_tp; i++) {
      //   if (counter[i] != counter[i - 1] + 1) {
      //     std::cerr << "Gap within package: counter " << counter[i - 1] << " -> " << counter[i]
      //               << " (size of gap: " << counter[i] - counter[i - 1] - 1 << ")" << std::endl;
      //   }
      // }

      // Check for gaps between packages
      // if (counter[0] != prev_counter + num_tp) {
      //   std::cerr << "Gap between packages: counter " << prev_counter << " -> " << counter[0] << std::endl;
      // }
      // prev_counter = counter[0];
    }
  }
};
