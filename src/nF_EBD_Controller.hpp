#pragma once

#include <iostream>

class nF_EBD_Controller {
 public:
  nF_EBD_Controller(char *serialNumberString);
  ~nF_EBD_Controller();
  void init();
  double read();
  void move_to(double value);
  void close();
  void autozero();

 private:
  int iD;
  char serialNumberString[1024];
  char name[1024];
  double offset = 1.0; // um
};