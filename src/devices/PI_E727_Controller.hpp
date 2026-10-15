#pragma once

#include <array>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// PI E-727 tip/tilt piezo controller (2 axes), connected via USB.
// Positions and targets are in mrad. Index 0 is the controller axis "1", and index 1 is the controller axis "2".
class PI_E727_Controller {
 public:
  explicit PI_E727_Controller(const char *serial_number);
  ~PI_E727_Controller();
  bool init();

  // Position of both axes (mrad), in one USB round trip.
  std::array<double, 2> read();

  // Sets a closed-loop target (mrad) and returns immediately. A worker thread or the next read() sends the target,
  // whichever gets the connection first. Targets that arrive before the send replace each other, and the last target
  // is always sent. Targets outside the range are clamped.
  void move_to(std::array<double, 2> target);

  // The range of the targets of each axis (mrad), in the units of read() and move_to().
  std::array<double, 2> range_min() const { return {min_pos, min_pos}; }
  std::array<double, 2> range_max() const { return {max_pos, max_pos}; }

  void close();
  void autozero();
  int autozero_axis(int ID, const std::string axis);

 private:
  int iD = -1;
  char serialNumberString[1024];
  char name[1024];
  double min_pos = 0.0;  // mrad
  double max_pos = 2.0;  // mrad
  std::mutex io_mutex;   // one command at a time on the connection

  std::thread mover;
  std::mutex target_mutex;
  std::condition_variable target_cv;
  std::array<double, 2> pending_target{};
  bool target_pending = false;
  bool stop_mover = false;

  void move_loop();
  void send_pending_target();
};
