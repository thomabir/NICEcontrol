#pragma once

#include <iostream>
#include <atomic>

class nF_EBD_Controller {
 public:
  nF_EBD_Controller(char *com_name);
  void init();
  double read();
  void move_to(double x_target, double y_target);
  void move_to_x(double x_target);
  void move_to_y(double y_target);
  double read_x();
  double read_y();
  void close();

 private:
  char name[1024];
  char com_name[1024];
  int fd; // file descriptor, to send commands to the stage
  double offset = 4.0;  // mrad
  std::atomic<bool> is_moving; // stage is unreachable while moving
  void move_to_blocking(float x_target, float y_target);
  void move_to_x_blocking(float x_target);
  void move_to_y_blocking(float y_target);
};