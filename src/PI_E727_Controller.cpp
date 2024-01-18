#include "PI_E727_Controller.hpp"

#include <cstring>

#include "../lib/pi/AutoZeroSample.h"
#include "../lib/pi/PI_GCS2_DLL.h"

PI_E727_Controller::PI_E727_Controller(char *serialNumberString) {
  // set serial number
  std::strcpy(this->serialNumberString, serialNumberString);

  // set name
  std::strcpy(this->name, "PI E-727 Tip/Tilt Piezo Controller S/N ");
  std::strcat(this->name, serialNumberString);
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

void PI_E727_Controller::move_to_x(double value) {
  const double dValue = value + offset;
  PI_MOV(iD, "2", &dValue);
}

void PI_E727_Controller::move_to_y(double value) {
  const double dValue = value + offset;
  PI_MOV(iD, "1", &dValue);
}

// run autozero procedure
void PI_E727_Controller::autozero() {
  // axis 1
  if (!AutoZeroIfNeeded(this->iD, "1")) {
    std::cout << "Autozero axis 1 failed" << std::endl;
  }

  // axis 2
  if (!AutoZeroIfNeeded(this->iD, "2")) {
    std::cout << "Autozero axis 2 failed" << std::endl;
  }
}

void PI_E727_Controller::close() { PI_CloseConnection(iD); }

// stuff for tip/tilt stage
bool AutoZeroIfNeeded(int ID, const std::string axis) {
  BOOL bAutoZeroed;

  if (!PI_qATZ(ID, axis.c_str(), &bAutoZeroed)) {
    return false;
  }

  if (!bAutoZeroed) {
    // if needed, autozero the axis
    std::cout << "AutoZero axis " << axis << "..." << std::endl;

    BOOL bUseDefaultVoltageArray[1];
    bUseDefaultVoltageArray[0] = TRUE;

    if (!PI_ATZ(ID, axis.c_str(), NULL, bUseDefaultVoltageArray)) {
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

  std::cout << "AutoZero finished successfully" << std::endl;

  return true;
}