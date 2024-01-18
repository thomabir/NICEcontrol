#include "MCL_NanoDrive.hpp"

#include <iostream>

#include "../lib/madlib/madlib.h"

MCL_OPDStage::MCL_OPDStage() {}

void MCL_OPDStage::init() {
  // print startup
  std::cout << "Initialising OPD stage" << std::endl;
  handle = MCL_InitHandle();

  // if handle is 0: stage is not connected.
  if (handle == 0) {
    std::cout << "OPD stage not connected" << std::endl;
    return;
  }

  // move to zero position
  this->move_to(0.0f);

  std::cout << "OPD stage initialised" << std::endl;
}

MCL_OPDStage::~MCL_OPDStage() {
  // move to zero position
  this->move_to(0.0f);
  std::cout << "OPD stage closed" << std::endl;
}

void MCL_OPDStage::move_to(float setpoint) {
  // move to setpoint + offset to give some headroom in both directions
  MCL_SingleWriteN(setpoint + this->offset, 3, handle);
}

double MCL_OPDStage::read() { return MCL_SingleReadN(3, handle) - this->offset; }
