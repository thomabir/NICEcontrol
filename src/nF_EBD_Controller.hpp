#pragma once

#include <array>
#include <atomic>
#include <iostream>

class nF_EBD_Controller {
 public:
  nF_EBD_Controller(const char *com_name);
  void init();
  std::array<float, 2> read();
  void move_to(std::array<float, 2> target);
  void close();

 private:
  char name[1024];
  char com_name[1024];
  int fd;                       // file descriptor, to send commands to the stage
  int axis0 = 0;                // x-axis
  int axis1 = 1;                // y-axis
  float offset = 1000.0;        // urad
  std::atomic<bool> is_moving;  // stage is unreachable while moving
  void move_to_blocking(float x_target, float y_target);
};
