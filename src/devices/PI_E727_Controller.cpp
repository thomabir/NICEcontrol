#include "devices/PI_E727_Controller.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <thread>

#include "devices/ScopedStdoutSilence.hpp"
#include "lib/pi/AutoZeroSample.h"
#include "lib/pi/PI_GCS2_DLL.h"

PI_E727_Controller::PI_E727_Controller(const char *serial_number) {
  std::strcpy(this->serialNumberString, serial_number);
  std::strcpy(this->name, "PI E-727 Tip/Tilt Piezo Controller S/N ");
  std::strcat(this->name, serial_number);
}

PI_E727_Controller::~PI_E727_Controller() { close(); }

bool PI_E727_Controller::init() {
  // Connect to the piezo controller, suppress noisy output from lib during connection
  {
    ScopedStdoutSilence silence;
    iD = PI_ConnectUSB(this->serialNumberString);
  }

  if (!PI_IsConnected(iD)) {
    std::cout << this->name << ": Connection failed" << std::endl;
    return false;
  }
  std::cout << this->name << ": Connection successful" << std::endl;

  int iError = 0;
  PI_qERR(iD, &iError);
  if (iError != 0) {
    std::cout << this->name << ": Error: " << iError << std::endl;
  }

  const int iEnable = 1;
  PI_SVO(iD, "1", &iEnable);
  PI_SVO(iD, "2", &iEnable);

  int iServoStatus = 0;
  PI_qSVO(iD, "1", &iServoStatus);
  if (iServoStatus != 1) {
    std::cout << this->name << ": Error: Cannot turn on servo on axis 1" << std::endl;
  }
  PI_qSVO(iD, "2", &iServoStatus);
  if (iServoStatus != 1) {
    std::cout << this->name << ": Error: Cannot turn on servo on axis 2" << std::endl;
  }

  {
    std::scoped_lock lock(this->target_mutex);
    this->stop_mover = false;
    this->target_pending = false;
  }
  this->mover = std::thread(&PI_E727_Controller::move_loop, this);
  return true;
}

std::array<double, 2> PI_E727_Controller::read() {
  std::scoped_lock lock(this->io_mutex);
  send_pending_target();
  double value[2] = {0.0, 0.0};
  PI_qPOS(iD, "1 2", value);
  return {value[0] * 1e-3, value[1] * 1e-3};  // the controller uses urad
}

void PI_E727_Controller::move_to(std::array<double, 2> target) {
  {
    std::scoped_lock lock(this->target_mutex);
    this->pending_target = target;
    this->target_pending = true;
  }
  this->target_cv.notify_one();
}

void PI_E727_Controller::move_loop() {
  while (true) {
    {
      std::unique_lock lock(this->target_mutex);
      this->target_cv.wait(lock, [this] { return this->target_pending || this->stop_mover; });
      if (this->stop_mover) {
        return;
      }
    }
    std::scoped_lock lock(this->io_mutex);
    send_pending_target();
  }
}

// Sends the pending target, if there is one. The caller holds io_mutex.
void PI_E727_Controller::send_pending_target() {
  std::array<double, 2> target;
  {
    std::scoped_lock lock(this->target_mutex);
    if (!this->target_pending) {
      return;
    }
    target = this->pending_target;
    this->target_pending = false;
  }
  double value[2];
  for (int i = 0; i < 2; i++) {
    value[i] = std::clamp(target[i], min_pos, max_pos) * 1e3;  // the controller uses urad
  }
  PI_MOV(iD, "1 2", value);
}

// run autozero procedure
void PI_E727_Controller::autozero() {
  autozero_axis(iD, "1");
  autozero_axis(iD, "2");
}

void PI_E727_Controller::close() {
  if (this->mover.joinable()) {
    {
      std::scoped_lock lock(this->target_mutex);
      this->stop_mover = true;
    }
    this->target_cv.notify_one();
    this->mover.join();
  }
  std::scoped_lock lock(this->io_mutex);
  if (iD >= 0) {
    PI_CloseConnection(iD);
    iD = -1;
  }
}

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
