#include "nF_EBD_Controller.hpp"

#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>

#include "../lib/nF/nF_interface.h"

nF_EBD_Controller::nF_EBD_Controller(char *com_name) {
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
  }
  this->fd = fd;

  // activate servo mode
  int status;
  int axis0 = 0;
  int axis1 = 1;
  nF_set_dev_axis_svo_m(this->fd, 1, &axis0, &status);
  nF_set_dev_axis_svo_m(this->fd, 1, &axis1, &status);

  // check if both axes servo modes are enabled. Print error if at least one is not enabled.
  int servo_mode;
  nF_get_dev_axis_svo_m(this->fd, 1, &axis0, &servo_mode);
  if (servo_mode != 1) {
    std::cout << this->name << ": Error: Servo mode not enabled for axis 0" << std::endl;
  }
  nF_get_dev_axis_svo_m(this->fd, 1, &axis1, &servo_mode);
  if (servo_mode != 1) {
    std::cout << this->name << ": Error: Servo mode not enabled for axis 1" << std::endl;
  }
}

void nF_EBD_Controller::move_to(double x_target, double y_target) {
  if (is_moving.load()) {
    return;
  }  // stage is unreachable while moving

  // recalculate target positions: rotate 45 degrees
  double x_target_r = +x_target + y_target;
  double y_target_r = +y_target - x_target;

  is_moving.store(true);
  std::thread([this, x_target_r, y_target_r] {
    move_to_blocking(x_target_r, y_target_r);
    is_moving.store(false);
  }).detach();
}

void nF_EBD_Controller::move_to_x(double x_target) {
  if (is_moving.load()) {
    return;
  }  // stage is unreachable while moving

  is_moving.store(true);
  std::thread([this, x_target] {
    move_to_x_blocking(x_target);
    is_moving.store(false);
  }).detach();
}

void nF_EBD_Controller::move_to_y(double y_target) {
  if (is_moving.load()) {
    return;
  }  // stage is unreachable while moving

  is_moving.store(true);
  std::thread([this, y_target] {
    move_to_y_blocking(y_target);
    is_moving.store(false);
  }).detach();
}

double nF_EBD_Controller::read_x() {
  int axis0 = 0;
  float position;
  nF_get_dev_axis_position_m(this->fd, 1, &axis0, &position);
  return position - this->offset;
}

double nF_EBD_Controller::read_y() {
  int axis1 = 1;
  float position;
  nF_get_dev_axis_position_m(this->fd, 1, &axis1, &position);
  return position - this->offset;
}

void nF_EBD_Controller::move_to_blocking(float x_target, float y_target) {
  int axis0 = 0;
  int axis1 = 1;
  x_target = x_target + this->offset;
  y_target = y_target + this->offset;
  nF_set_dev_axis_target_m(this->fd, 1, 1, &axis0, &x_target);
  nF_set_dev_axis_target_m(this->fd, 1, 1, &axis1, &y_target);
}

void nF_EBD_Controller::move_to_x_blocking(float x_target) {
  int axis0 = 0;
  x_target = x_target + this->offset;
  nF_set_dev_axis_target_m(this->fd, 1, 1, &axis0, &x_target);
}

void nF_EBD_Controller::move_to_y_blocking(float y_target) {
  int axis1 = 1;
  y_target = y_target + this->offset;
  nF_set_dev_axis_target_m(this->fd, 1, 1, &axis1, &y_target);
}

void nF_EBD_Controller::close() { nF_intf_disconnect(this->fd); }
