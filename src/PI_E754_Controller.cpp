#include "PI_E754_Controller.hpp"

#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>

#include "../lib/pi/AutoZeroSample.h"
#include "../lib/pi/PI_GCS2_DLL.h"

PI_E754_Controller::PI_E754_Controller(char *serialNumberString) {
  // set serial number
  std::strcpy(this->serialNumberString, serialNumberString);

  // set name
  std::strcpy(this->name, "PI E-754 Linear Piezo Controller S/N ");
  std::strcat(this->name, serialNumberString);

  // set is_moving to false
  is_moving = false;
}

PI_E754_Controller::~PI_E754_Controller() { close(); }

void PI_E754_Controller::init() {
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

  // enable servo
  const int iEnable = 1;
  PI_SVO(iD, "1", &iEnable);

  // check if servo is enabled.
  int iServoStatus = 0;
  PI_qSVO(iD, "1", &iServoStatus);
  if (iServoStatus != 1) {
    std::cout << this->name << ": Error: Cannot turn on servo on axis 1" << std::endl;
  }
}

double PI_E754_Controller::read() {
  double value = 0;
  PI_qPOS(iD, "1", &value);
  return value * 1e3 - offset;  // convert to nm
}

void PI_E754_Controller::move_to(double value) {
  if (is_moving.load()) {
    return;
  }  // stage is unreachable while moving

  is_moving.store(true);
  std::thread([this, value] {
    move_to_blocking(value);
    is_moving.store(false);
  }).detach();
}

void PI_E754_Controller::move_to_blocking(double value) {
  double dValue = (value + offset) * 1e-3;  // convert to um

  // check if the value is within the allowed range, clamp if necessary
  if (dValue < min_pos) {
    // std::cout << this->name << ": Warning: Cannot go that far. Commanded position is " << dValue << " um, going to "
    // << min_pos << " um instead." << std::endl;
    dValue = min_pos;
  } else if (dValue > max_pos) {
    // std::cout << this->name << ": Warning: Cannot go that far. Commanded position is " << dValue << " um, going to "
    // << max_pos << " um instead." << std::endl;
    dValue = max_pos;
  }

  PI_MOV(iD, "1", &dValue);  // PI_MOV blocks until the move is done
}

// run autozero procedure
void PI_E754_Controller::autozero() { autozero_axis(iD, "1"); }

void PI_E754_Controller::close() { PI_CloseConnection(iD); }

int PI_E754_Controller::autozero_axis(int ID, const std::string axis) {
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
