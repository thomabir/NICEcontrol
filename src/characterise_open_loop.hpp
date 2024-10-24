#pragma once

#include <chrono>
#include <fstream>

#include "SensorData.hpp"
#include "TSCircularBuffer.hpp"

template <class C, class A>
void characterise_open_loop(SISOControlLoop<C, A> &loop, TSCircularBuffer<SensorData> &sensorDataQueue, float P,
                            float I, float t_settle, float t_record, std::string filename) {
  // prepare storage file
  std::ofstream file(filename);
  file << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
          "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

  loop.reset_all();
  loop.control_mode.store(0);
  loop.p.store(P);
  loop.i.store(I);
  std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));  // wait settling time
  while (!sensorDataQueue.isempty()) {
    sensorDataQueue.pop();
  }  // flush measurement queue

  // record data for t_record seconds
  auto t_start = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
         t_record) {
    // wait 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // write to file
    if (!sensorDataQueue.isempty()) {
      int N = sensorDataQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = sensorDataQueue.pop();
        // format: 6 decimals for time, 3 decimals for measurement
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
             << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
             << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
             << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
             << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
             << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
      }
    }
  }
  // reset
  loop.reset_all();
};