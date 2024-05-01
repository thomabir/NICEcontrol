#include "PI_E727_Controller.hpp"

#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>

#include "../lib/pi/AutoZeroSample.h"
#include "../lib/pi/PI_GCS2_DLL.h"

PI_E727_Controller::PI_E727_Controller(char *serialNumberString) {
  // set serial number
  std::strcpy(this->serialNumberString, serialNumberString);

  // set name
  std::strcpy(this->name, "PI E-727 Tip/Tilt Piezo Controller S/N ");
  std::strcat(this->name, serialNumberString);

  // set is_moving to false
  is_moving_x = false;
  is_moving_y = false;
}

PI_E727_Controller::~PI_E727_Controller() { close(); }

void PI_E727_Controller::init() {
  // Connect to the piezo controller

  // PI writes very verbose messages to stdout, so we temporarilly redirect stdout to /dev/null
  // I am aware this is an ugly hack, but I haven't found a better way.
  // Probably not thread-safe.
  fclose(stdout);
  iD = PI_ConnectUSB(this->serialNumberString);
  freopen("/dev/tty", "w", stdout);

  // Check if connection was successful
  if (PI_IsConnected(iD)) {
    std::cout << this->name << ": Connection successful" << std::endl;
  } else {
    std::cout << this->name << ": Connection failed" << std::endl;
    return;
  }

  // find available axes
  // char szAxes[1024];
  // PI_qSAI(iD, szAxes, 1024);
  // std::cout << "Available axes: " << szAxes << std::endl;

  // find units that qPOS and MOV use
  // char szUnits[1024];
  // PI_qPUN(iD, "2", szUnits, 1024);
  // std::cout << "Units: " << szUnits << std::endl;

  // If an error occurred, print it to the console
  int iError = 0;
  PI_qERR(iD, &iError);
  if (iError != 0) {
    std::cout << this->name << ": Error: " << iError << std::endl;
  }

  // set servo mode
  const int iEnable = 1;
  PI_SVO(iD, "1", &iEnable);
  PI_SVO(iD, "2", &iEnable);

  // check if both axes servo modes are enabled. Print error if at least one is not enabled.
  int iServoStatus = 0;
  PI_qSVO(iD, "1", &iServoStatus);
  if (iServoStatus != 1) {
    std::cout << this->name << ": Error: Cannot turn on servo on axis 1" << std::endl;
  }
  PI_qSVO(iD, "2", &iServoStatus);
  if (iServoStatus != 1) {
    std::cout << this->name << ": Error: Cannot turn on servo on axis 2" << std::endl;
  }
}

double PI_E727_Controller::readx() {
  double value = 0;
  PI_qPOS(iD, "2", &value);
  return value - offset;
}

double PI_E727_Controller::ready() {
  double value = 0;
  PI_qPOS(iD, "1", &value);
  return value - offset;
}

void PI_E727_Controller::move_to_axis(int axis, double value) {
  if (axis == 1) {
    move_to_x(value);
  } else if (axis == 2) {
    move_to_y(value);
  } else {
    std::cout << "Invalid axis" << std::endl;
  }
}

void PI_E727_Controller::move_to_x(double value) {
  if (is_moving_x.load()) {return;} // stage is unreachable while moving

  is_moving_x.store(true);
  std::thread([this, value] {
    move_to_x_blocking(value);
    is_moving_x.store(false);
  }).detach();

}

void PI_E727_Controller::move_to_y(double value) {
  if (is_moving_y.load()) {return;} // stage is unreachable while moving

  is_moving_y.store(true);
  std::thread([this, value] {
    move_to_y_blocking(value);
    is_moving_y.store(false);
  }).detach();

}

void PI_E727_Controller::move_to_x_blocking(double value) {
  const double dValue = - value + offset; // the negative sign is because of the orientation of the piezo
  PI_MOV(iD, "1", &dValue);
}

void PI_E727_Controller::move_to_y_blocking(double value) {
  const double dValue = value + offset;
  PI_MOV(iD, "2", &dValue);
}

// run autozero procedure
void PI_E727_Controller::autozero() {
  autozero_axis(iD, "1");
  autozero_axis(iD, "2");
}

void PI_E727_Controller::close() { PI_CloseConnection(iD); }

int PI_E727_Controller::autozero_axis(int ID, const std::string axis) {
  BOOL bAutoZeroed;

  if (!PI_qATZ(ID, axis.c_str(), &bAutoZeroed)) {
    return false;
  }

  if (!bAutoZeroed) {
    // if needed, autozero the axis
    std::cout << "Starting AutoZero axis " << axis << "..." << std::endl;

    BOOL bUseDefaultVoltageArray[1];
    bUseDefaultVoltageArray[0] = TRUE;

    if (!PI_ATZ(ID, axis.c_str(), NULL, bUseDefaultVoltageArray)) {
      std::cout << "AutoZero axis " << axis << " failed" << std::endl;
      return false;
    }

    // Wait until the autozero move is done.
    BOOL bFlag = FALSE;

    while (bFlag != TRUE) {
      if (!PI_IsControllerReady(ID, &bFlag)) {
        return false;
      }
    }
  }

  std::cout << "AutoZero axis " << axis << " finished successfully" << std::endl;

  return true;
}