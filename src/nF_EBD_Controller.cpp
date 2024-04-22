#include "nF_EBD_Controller.hpp"

#include <cstring>

#include "../lib/nF/nF_interface.h"

nF_EBD_Controller::nF_EBD_Controller(char *serialNumberString) {
}

nF_EBD_Controller::~nF_EBD_Controller() { close(); }

void nF_EBD_Controller::init() {
  // TODO test connection

  int fd=-1;

  char ipAddr[16] = "192.168.168.168"; // IP address of the controller

  printf("Using USB interface (RNDIS), IP address: %s\n", ipAddr);
  fd = nF_intf_connect_tcpip(ipAddr);

  if (fd < 0) {
    printf("Error: Could not connect to controller\n");
    return;
  }

  // print fd
  printf("fd: %d\n", fd);
}

double nF_EBD_Controller::read() {
  double value = 0;
  // TODO: read value from controller
  return value - offset;
}

void nF_EBD_Controller::move_to(double value) {
  double dValue = value + offset;
  // TODO: move
}

// run autozero procedure
void nF_EBD_Controller::autozero() {
  // TODO: autozero
}

void nF_EBD_Controller::close() { 
  // TODO: close connection
}
