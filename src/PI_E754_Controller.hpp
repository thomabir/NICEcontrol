#pragma once

#include <iostream>

class PI_E754_Controller {
 public:
  PI_E754_Controller(char *serialNumberString);
  ~PI_E754_Controller();
  void init();
  double read();
  void move_to(double value);
  void close();
  void autozero();
  int autozero_axis(int ID, const std::string axis);

 private:
  int iD;
  char serialNumberString[1024];
  char name[1024];
  double offset = 1.0; // um
};