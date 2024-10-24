#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "characterise_open_loop.hpp"
#include "utils.hpp"

template <class C1, class A1, class C2, class A2, class A3>
void characterise_joint_closed_loop(SISOControlLoop<C1, A1> &opd_loop, SISOControlLoop<C2, A2> &shear_x1_loop,
                                    SISOControlLoop<C2, A2> &shear_x2_loop, SISOControlLoop<C2, A3> &shear_y1_loop,
                                    SISOControlLoop<C2, A3> &shear_y2_loop,
                                    TSCircularBuffer<SensorData> &sensorDataQueue, float opd_p, float opd_i,
                                    float shear_p, float shear_i, float t_settle, float t_record,
                                    std::string description) {
  std::cout << "Starting joint closed loop characterisation" << std::endl;

  // reset control loops
  opd_loop.reset_all();
  shear_x1_loop.reset_all();
  shear_x2_loop.reset_all();
  shear_y1_loop.reset_all();
  shear_y2_loop.reset_all();

  // set control parameters
  opd_loop.p.store(opd_p);
  opd_loop.i.store(opd_i);

  shear_x1_loop.p.store(shear_p);
  shear_x1_loop.i.store(shear_i);

  shear_x2_loop.p.store(shear_p);
  shear_x2_loop.i.store(shear_i);

  shear_y1_loop.p.store(shear_p);
  shear_y1_loop.i.store(shear_i);

  shear_y2_loop.p.store(shear_p);
  shear_y2_loop.i.store(shear_i);

  // directory to store files in: dire name is opd_date_time (e.g. measurements/opd_2021-09-01_12:00:00)
  auto datetime_string = utils::get_iso_datestring();
  std::string dirname = "measurements/" + datetime_string + "_" + description;
  std::filesystem::create_directory(dirname);

  // CLOSED LOOP CHARACTERISATION
  std::cout << "\t Closed loop time series" << std::endl;
  // storage file
  std::string filename = dirname + "/closed_loop.csv";
  std::ofstream file(filename);

  file << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
          "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

  // reset controller
  opd_loop.controller.reset_state();
  shear_x1_loop.controller.reset_state();
  shear_x2_loop.controller.reset_state();
  shear_y1_loop.controller.reset_state();
  shear_y2_loop.controller.reset_state();

  // turn on control loops
  opd_loop.control_mode.store(4);
  shear_x1_loop.control_mode.store(4);
  shear_x2_loop.control_mode.store(4);
  shear_y1_loop.control_mode.store(4);
  shear_y2_loop.control_mode.store(4);

  // wait settling time
  std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

  // flush data queues
  while (!sensorDataQueue.isempty()) {
    sensorDataQueue.pop();
  }

  // record for recording time: every 10 ms, write contents to file
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
        // format: 6 decimals for time, 3 decimals for OPD
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
             << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
             << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
             << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
             << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
             << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
      }
    }
  }

  file.close();

  //  turn off all control loops
  opd_loop.control_mode.store(0);
  shear_x1_loop.control_mode.store(0);
  shear_x2_loop.control_mode.store(0);
  shear_y1_loop.control_mode.store(0);
  shear_y2_loop.control_mode.store(0);

  // DONE
  std::cout << "Finished joint closed loop characterisation: " << description << std::endl;
};