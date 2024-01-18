#pragma once

class MCL_OPDStage {
 public:
  MCL_OPDStage();
  ~MCL_OPDStage();
  void init();
  void move_to(float setpoint);
  double read();

 private:
  int handle;
  double offset = 20.0;  // um
};