#include "nF_EBD_Controller.hpp"

#include <array>
#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>

#include "../lib/nF/nF_interface.h"

nF_EBD_Controller::nF_EBD_Controller(const char *com_name) {
  // set com_name
  std::strcpy(this->com_name, com_name);

  // set name: nF EBD Controller + com port
  std::strcpy(this->name, "nF EBD Controller ");
  std::strcat(this->name, com_name);

  // set is_moving to false
  is_moving = false;
}

void nF_EBD_Controller::init() {
  // connect to the controller
  int fd = nF_intf_connect_tty(this->com_name);
  if (fd < 0) {
    std::cout << this->name << ": Connection failed" << std::endl;
    return;
  } else if (fd == 1) {
    std::cout << this->name << ": Connection successful" << std::endl;
    return;
  }
  std::cout << this->name << ": fd returned by nF_intf_connect_tty: " << fd << std::endl;
  this->fd = fd;

  // activate servo mode
  int status;
  nF_set_dev_axis_svo_m(this->fd, 1, &this->axis0, &status);
  nF_set_dev_axis_svo_m(this->fd, 1, &this->axis1, &status);

  // check if both axes servo modes are enabled. Print error if at least one is not enabled.
  int servo_mode;
  nF_get_dev_axis_svo_m(this->fd, 1, &this->axis0, &servo_mode);
  if (servo_mode != 1) {
    std::cout << this->name << ": Error: Servo mode not enabled for axis 0" << std::endl;
  }
  nF_get_dev_axis_svo_m(this->fd, 1, &this->axis1, &servo_mode);
  if (servo_mode != 1) {
    std::cout << this->name << ": Error: Servo mode not enabled for axis 1" << std::endl;
  }
}

void nF_EBD_Controller::move_to(std::array<double, 2> target) {
  if (is_moving.load()) {
    return;
  }  // stage is unreachable while moving

  // recalculate target positions: rotate 45 degrees
  double x = target[0];  //+ target[1];
  double y = target[1];  //- target[0];

  is_moving.store(true);
  std::thread([this, x, y] {
    move_to_blocking(x, y);
    is_moving.store(false);
  }).detach();
}

std::array<double, 2> nF_EBD_Controller::read() {
  float position_x, position_y;
  nF_get_dev_axis_position_m(this->fd, 1, &this->axis0, &position_x);
  nF_get_dev_axis_position_m(this->fd, 1, &this->axis1, &position_y);
  return {position_x * 1e3 - this->offset, position_y * 1e3 - this->offset};
}

void nF_EBD_Controller::move_to_blocking(double x_target, double y_target) {
  x_target = (x_target + this->offset) * 1e-3;
  y_target = (y_target + this->offset) * 1e-3;

  // nF API expects float, not double
  float x_target_f = static_cast<float>(x_target);
  float y_target_f = static_cast<float>(y_target);

  nF_set_dev_axis_target_m(this->fd, 1, 1, &this->axis0, &x_target_f);
  nF_set_dev_axis_target_m(this->fd, 1, 1, &this->axis1, &y_target_f);
}

void nF_EBD_Controller::close() { nF_intf_disconnect(this->fd); }
