#pragma once

#include <iostream>
#include <atomic>

class nF_EBD_Controller {
 public:
  nF_EBD_Controller();
  void move_to(double x1_target, double y1_target, double x2_target, double y2_target);

 private:
  double offset = 4.0;  // mrad
  std::atomic<bool> is_moving; // stage is unreachable while moving
  void move_to_blocking(double x1_target, double y1_target, double x2_target, double y2_target);
};