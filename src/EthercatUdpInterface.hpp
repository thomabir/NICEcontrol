#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>

/**
 * @brief A class to interface with an EtherCAT master via UDP
 *
 * This class provides functionality to send control commands to an EtherCAT master
 * using UDP sockets. The commands include an OPD setpoint value and two control flags.
 */
class EthercatUdpInterface {
 public:
  /**
   * @brief Construct a new EthercatUdpInterface object
   *
   * @param ip The IP address of the EtherCAT master
   * @param port The UDP port to use for communication
   */
  EthercatUdpInterface(const char* ip, int port) : ip_(ip), port_(port) {
    // Initialize socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      std::cerr << "Failed to create socket for EtherCAT UDP interface" << std::endl;
      return;
    }

    // Set up destination address
    memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(port_);

    // Convert IP address from string to binary form
    if (inet_pton(AF_INET, ip_, &dest_addr_.sin_addr) <= 0) {
      std::cerr << "Invalid IP address: " << ip_ << std::endl;
      close(sockfd_);
      sockfd_ = -1;
    }

    // print IP address and port
  }

  /**
   * @brief Destroy the EthercatUdpInterface object and close socket
   */
  ~EthercatUdpInterface() {
    if (sockfd_ >= 0) {
      close(sockfd_);
    }
  }

  /**
   * @brief Send control commands to the EtherCAT master
   *
   * Sends a 5-byte packet to the EtherCAT master:
   * - 4 bytes: OPD setpoint as float
   * - 1 byte: control flags (reset_phase_unwrap and run_control_loop)
   *
   * @param opd_setpoint The OPD setpoint value (in nm)
   * @param reset_phase_unwrap Flag to reset phase unwrapping
   * @param run_control_loop Flag to enable/disable control loop
   * @return true if sending was successful, false otherwise
   */
  bool send_commands(float opd_setpoint, float p, float i, bool reset_phase_unwrap, bool run_control_loop) {
    if (sockfd_ < 0) {
      std::cerr << "Cannot send commands: socket not initialized" << std::endl;
      return false;
    }

    // Prepare the 5-byte packet
    char buffer[4 + 1 + 4 + 4];  // 4 x 3 for floats (setpoint, p, i) + 1 for control byte

    // Convert float to binary representation and copy to buffer
    memcpy(buffer, &opd_setpoint, sizeof(float));

    // Create control byte: bit 0 = run_control_loop, bit 1 = reset_phase_unwrap
    unsigned char control_byte = 0;
    if (run_control_loop) control_byte |= 0x01;
    if (reset_phase_unwrap) control_byte |= 0x02;

    buffer[4] = control_byte;

    memcpy(buffer + 5, &p, sizeof(float));
    memcpy(buffer + 9, &i, sizeof(float));

    // Send the packet
    ssize_t sent_bytes = sendto(sockfd_, buffer, sizeof(buffer), 0, (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));

    if (sent_bytes != sizeof(buffer)) {
      std::cerr << "Failed to send complete packet to EtherCAT master" << std::endl;
      return false;
    }

    return true;
  }

 private:
  const char* ip_;                ///< IP address of EtherCAT master
  int port_;                      ///< UDP port for communication
  int sockfd_ = -1;               ///< Socket file descriptor
  struct sockaddr_in dest_addr_;  ///< Destination address structure
};
