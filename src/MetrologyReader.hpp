#include <arpa/inet.h>   // ethernet
#include <netinet/in.h>  // ethernet
#include <sys/socket.h>  // ethernet

#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <numbers>
#include <thread>

#include "SharedResources.hpp"
#include "utils.hpp"

#pragma once

class MetrologyReader {
 public:
  // constructor: take reference to metrology resources as input
  MetrologyReader(MetrologyResources &met_res) : met_res(met_res) { sockfd = setup_ethernet(); }

  // Destructor to clean up resources
  ~MetrologyReader() {
    stop();
    if (sockfd >= 0) {
      close(sockfd);
    }
  }

  // Start the measurement thread
  void start() {
    if (!calculation_thread.joinable()) {
      running.store(true);
      calculation_thread = std::thread(&MetrologyReader::run_calculation, this);
    }
  }

  // Stop the measurement thread
  void stop() {
    if (calculation_thread.joinable()) {
      running.store(false);
      calculation_thread.join();
    }
  }

 private:
  MetrologyResources &met_res;
  int sockfd = -1;  // socket for UDP
  std::thread calculation_thread;
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
    int count = 0;
    int buffer_size = 1024;
    char buffer[buffer_size];

    Iir::Butterworth::LowPass<2> shear_x1_lpfilt, shear_x2_lpfilt, shear_y1_lpfilt, shear_y2_lpfilt;
    Iir::Butterworth::LowPass<2> point_x1_lpfilt, point_x2_lpfilt, point_y1_lpfilt, point_y2_lpfilt;
    Iir::Butterworth::LowPass<2> opd_lpfilt;
    const float opd_samplingrate = 128000.;
    const float opd_lpfilt_cutoff = 1000.;
    const float shear_samplingrate = 128000.;
    const float shear_lpfilt_cutoff = 300.;

    // setup filters
    shear_x1_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
    shear_x2_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
    shear_y1_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
    shear_y2_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);

    point_x1_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
    point_x2_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
    point_y1_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
    point_y2_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);

    opd_lpfilt.setup(opd_samplingrate, opd_lpfilt_cutoff);

    while (running.load()) {
      float shear_x1_f = 0., shear_x2_f = 0., shear_y1_f = 0., shear_y2_f = 0., point_x1_f = 0., point_x2_f = 0.,
            point_y1_f = 0., point_y2_f = 0., opd_nm = 0., sci_null = 0.;
      auto t = utils::getTime();

      // read the measurement from the ethernet connection
      count++;

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

      // Convert received data to vector of 10 ints
      const static int num_channels = 23;
      const static int num_timepoints = 10;
      int receivedDataInt[num_channels * num_timepoints];
      std::memcpy(receivedDataInt, buffer, sizeof(int) * num_channels * num_timepoints);

      static int counter[num_timepoints];
      static int adc_shear1[num_timepoints];
      static int adc_shear2[num_timepoints];
      static int adc_shear3[num_timepoints];
      static int adc_shear4[num_timepoints];
      static int adc_point1[num_timepoints];
      static int adc_point2[num_timepoints];
      static int adc_point3[num_timepoints];
      static int adc_point4[num_timepoints];
      // static int adc_pos_ref[num_timepoints];
      static int adc_sci_null[num_timepoints];
      static int adc_sci_ref[num_timepoints];
      static int adc_opd_back[num_timepoints];
      static int adc_opd_ref[num_timepoints];
      static int adc_opd_pll[num_timepoints];
      static float opd_rad[num_timepoints];
      static int32_t opd_int[num_timepoints];
      static float shear_x1_um[num_timepoints];
      static float shear_x2_um[num_timepoints];
      static float shear_y1_um[num_timepoints];
      static float shear_y2_um[num_timepoints];
      static float point_x1_um[num_timepoints];
      static float point_x2_um[num_timepoints];
      static float point_y1_um[num_timepoints];
      static float point_y2_um[num_timepoints];
      static float opd_rad_i_prev = 0.0f;

      // Process data for each timepoint
      for (int i = 0; i < num_timepoints; i++) {
        counter[i] = receivedDataInt[num_channels * i];

        adc_shear1[i] = receivedDataInt[num_channels * i + 1];
        adc_shear2[i] = receivedDataInt[num_channels * i + 2];
        adc_shear3[i] = receivedDataInt[num_channels * i + 3];
        adc_shear4[i] = receivedDataInt[num_channels * i + 4];
        adc_point1[i] = receivedDataInt[num_channels * i + 5];
        adc_point2[i] = receivedDataInt[num_channels * i + 6];
        adc_point3[i] = receivedDataInt[num_channels * i + 7];
        adc_point4[i] = receivedDataInt[num_channels * i + 8];
        // adc_pos_ref[i] = receivedDataInt[num_channels * i + 9];
        adc_opd_back[i] = receivedDataInt[num_channels * i + 9];
        adc_opd_ref[i] = receivedDataInt[num_channels * i + 10];
        adc_opd_pll[i] = receivedDataInt[num_channels * i + 11] * 128;  // ADU * 256
        opd_int[i] = receivedDataInt[num_channels * i + 12];
        opd_rad[i] =
            -float(opd_int[i]) * std::numbers::pi / (std::pow(2.0, 15) - 1.);    // phase (signed 16 bit int) -> rad
        shear_x1_um[i] = float(receivedDataInt[num_channels * i + 13]) / 3000.;  // um
        shear_x2_um[i] = float(receivedDataInt[num_channels * i + 14]) / 3000.;  // um
        shear_y1_um[i] = float(receivedDataInt[num_channels * i + 15]) / 3000.;  // um
        shear_y2_um[i] = float(receivedDataInt[num_channels * i + 16]) / 3000.;  // um
        point_x1_um[i] = float(receivedDataInt[num_channels * i + 17]) / 1000.;  // urad
        point_x2_um[i] = float(receivedDataInt[num_channels * i + 18]) / 1000.;  // urad
        point_y1_um[i] = float(receivedDataInt[num_channels * i + 19]) / 1000.;  // urad
        point_y2_um[i] = float(receivedDataInt[num_channels * i + 20]) / 1000.;  // urad
        adc_sci_null[i] = receivedDataInt[num_channels * i + 21];                // ADU
        adc_sci_ref[i] = receivedDataInt[num_channels * i + 22];                 // ADU
      }

      // phase-unwrap the OPD signal
      const int max_unwrap_iters = 500;
      static int unwrap_iters = 0;

      for (int i = 0; i < num_timepoints; i++) {
        if (met_res.reset_phase_unwrap.load()) {
          unwrap_iters = 0;
          opd_rad_i_prev = 0;
        }

        opd_rad[i] =
            opd_rad[i] + unwrap_iters * 2 * std::numbers::pi;  // add the current number of unwrapping iterations
        while ((opd_rad[i] - opd_rad_i_prev > std::numbers::pi) && (unwrap_iters > -max_unwrap_iters)) {
          opd_rad[i] -= 2 * std::numbers::pi;
          unwrap_iters--;
        }
        while ((opd_rad[i] - opd_rad_i_prev < -std::numbers::pi) && (unwrap_iters < max_unwrap_iters)) {
          opd_rad[i] += 2 * std::numbers::pi;
          unwrap_iters++;
        }
        opd_rad_i_prev = opd_rad[i];
      }

      static float opd_rad_f = 0.0f;
      // filter signals
      for (int i = 0; i < num_timepoints; i++) {
        // coordinate system of quad cell is rotated by 45 degrees, hence the combination of basis vectors
        shear_x1_f = shear_x1_lpfilt.filter(shear_y1_um[i] + shear_x1_um[i]);
        shear_x2_f = shear_x2_lpfilt.filter(shear_y2_um[i] + shear_x2_um[i]);
        shear_y1_f = shear_y1_lpfilt.filter(shear_x1_um[i] - shear_y1_um[i]);
        shear_y2_f = shear_y2_lpfilt.filter(shear_x2_um[i] - shear_y2_um[i]);

        point_x1_f = point_x1_lpfilt.filter(point_y1_um[i] + point_x1_um[i]);
        point_x2_f = point_x2_lpfilt.filter(point_y2_um[i] + point_x2_um[i]);
        point_y1_f = point_y1_lpfilt.filter(point_x1_um[i] - point_y1_um[i]);
        point_y2_f = point_y2_lpfilt.filter(point_x2_um[i] - point_y2_um[i]);

        opd_rad_f = opd_lpfilt.filter(opd_rad[i]);

        // derive null intensity from science signals
        sci_null = met_res.null_lockin.process(adc_sci_null[i], adc_sci_ref[i]);
      }

      // convert OPD from radians to nm
      opd_nm = opd_rad_f * 1550 / (2 * std::numbers::pi);

      // enqueue sensor data
      met_res.sensorDataQueue.push({t, opd_nm, shear_x1_f, shear_x2_f, shear_y1_f, shear_y2_f, point_x1_f, point_x2_f,
                                    point_y1_f, point_y2_f, sci_null});
      // std::cout << "Pushed data to queue: " << t << std::endl;

      // enqueue adc measurements
      for (int i = 0; i < num_timepoints; i++) {
        met_res.adc_queues[0].push({counter[i], adc_shear1[i]});
        met_res.adc_queues[1].push({counter[i], adc_shear2[i]});
        met_res.adc_queues[2].push({counter[i], adc_shear3[i]});
        met_res.adc_queues[3].push({counter[i], adc_shear4[i]});
        met_res.adc_queues[4].push({counter[i], adc_point1[i]});
        met_res.adc_queues[5].push({counter[i], adc_point2[i]});
        met_res.adc_queues[6].push({counter[i], adc_point3[i]});
        met_res.adc_queues[7].push({counter[i], adc_point4[i]});
        met_res.adc_queues[8].push({counter[i], adc_opd_ref[i]});
        met_res.adc_queues[9].push({counter[i], adc_opd_back[i]});
        met_res.adc_queues[10].push({counter[i], adc_shear1[i] + adc_shear2[i] + adc_shear3[i] + adc_shear4[i]});
        met_res.adc_queues[11].push({counter[i], adc_point1[i] + adc_point2[i] + adc_point3[i] + adc_point4[i]});
        met_res.adc_queues[12].push({counter[i], adc_sci_null[i]});
        met_res.adc_queues[13].push({counter[i], adc_sci_ref[i]});
        met_res.adc_queues[14].push({counter[i], adc_opd_pll[i]});
      }

      // Run control loops
      met_res.shear_x1_loop.control(t, shear_x1_f);
      met_res.shear_x2_loop.control(t, shear_x2_f);
      met_res.shear_y1_loop.control(t, shear_y1_f);
      met_res.shear_y2_loop.control(t, shear_y2_f);
      met_res.point_1_loop.control(t, {point_x1_f, point_y1_f});
      met_res.point_2_loop.control(t, {point_x2_f, point_y2_f});
    }
  }
};
