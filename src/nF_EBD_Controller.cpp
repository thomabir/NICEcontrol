#include "nF_EBD_Controller.hpp"

#include <cstring>
#include <cstdlib>
#include <sstream>
#include <thread>
#include <iostream>
#include <atomic>

// #include "../lib/nF/nF_interface.h"

nF_EBD_Controller::nF_EBD_Controller() {
  is_moving = false;
}

void nF_EBD_Controller::move_to(double x1_target, double y1_target, double x2_target, double y2_target) {
  if (is_moving.load()) {return;} // stage is unreachable while moving

  // recalculate target positions: rotate 45 degrees
  double x1_target_r = + x1_target + y1_target;
  double y1_target_r = + y1_target - x1_target;
  double x2_target_r = + x2_target + y2_target;
  double y2_target_r = + y2_target - x2_target;

  is_moving.store(true);
  std::thread([this, x1_target_r, y1_target_r, x2_target_r, y2_target_r] {
    move_to_blocking(x1_target_r, y1_target_r, x2_target_r, y2_target_r);
    is_moving.store(false);
  }).detach();

}

void nF_EBD_Controller::move_to_blocking(double x1_target, double y1_target, double x2_target, double y2_target) {
  std::stringstream nF_move_command;
  nF_move_command << "./nf_cli " << x1_target + offset << " " << y1_target + offset << " " << x2_target + offset << " " << y2_target + offset;
  std::system(nF_move_command.str().c_str()); // Convert the stringstream to string and then to C-style string
}
