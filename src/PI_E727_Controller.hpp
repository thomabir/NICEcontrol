#pragma once

#include <atomic>
#include <iostream>

class PI_E727_Controller {
 public:
  PI_E727_Controller(char *serialNumberString);
  ~PI_E727_Controller();
  void init();
  double readx();
  double ready();
  void move_to_axis(int axis, double value);
  void move_to_x(double value);
  void move_to_y(double value);
  void close();
  void autozero();
  int autozero_axis(int ID, const std::string axis);

 private:
  int iD;
  char serialNumberString[1024];
  char name[1024];
  double offset = 1000.0;  // urad
  double min_pos = 0.;  // urad
  double max_pos = 2000.;  // urad
  std::atomic<bool> is_moving_x;  // stage is unreachable while moving
  std::atomic<bool> is_moving_y;
  void move_to_x_blocking(double value);
  void move_to_y_blocking(double value);
};