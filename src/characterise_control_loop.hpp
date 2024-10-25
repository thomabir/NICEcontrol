#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "characterise_open_loop.hpp"
#include "utils.hpp"

template <class C, class A>
void characterise_control_loop(SISOControlLoop<C, A> &loop, TSCircularBuffer<SensorData> &sensorDataQueue, float P,
                               float I, float t_settle, float t_record, float f1, float f2, float fsteps,
                               float dither_amp, std::string description) {
  std::cout << "Starting control loop characterisation" << std::endl;

  // dither frequencies
  std::vector<float> dither_freqs = {0.1, 0.2, 0.4, 0.8};  // low frequencies take long, so only a few

  // append logarithmic freqs
  float logf1 = std::log10(f1);
  float logf2 = std::log10(f2);
  for (int i = 0; i < fsteps; i++) {
    dither_freqs.push_back(std::pow(10, logf1 + i * (logf2 - logf1) / (fsteps - 1)));
  }

  // dither amplitudes
  std::vector<float> dither_amps(dither_freqs.size(), dither_amp);  // nm

  // recording time: at least 1 s, at most 10/freq
  std::vector<float> recording_times(dither_freqs.size(), 0.0);  // s

  for (size_t i = 0; i < dither_freqs.size(); i++) {
    recording_times[i] = std::max(1.0, 10.0 / dither_freqs[i]);
  }

  // directory for storage is labelled with ISO datestring + description
  auto datetime_string = utils::get_iso_datestring();
  std::string dirname = "measurements/" + datetime_string + "_" + description;
  std::filesystem::create_directory(dirname);

  // write into parameters.txt: P, I, time, description
  std::string param_filename = dirname + "/parameters.txt";
  std::ofstream param_file(param_filename);
  param_file << "P: " << P << "\n";
  param_file << "I: " << I << "\n";
  param_file << "t_settle: " << t_settle << "\n";
  param_file << "t_record: " << t_record << "\n";
  param_file << "f1: " << f1 << "\n";
  param_file << "f2: " << f2 << "\n";
  param_file << "fsteps: " << fsteps << "\n";
  param_file << "dither_amp: " << dither_amp << "\n";
  param_file << "description: " << description << "\n";
  param_file.close();

  // write into freqs.csv: dither frequencies, dither amplitudes, recording times
  // dither frequency is a string that matches the filename
  std::string freq_filename = dirname + "/freqs.csv";
  std::ofstream freq_file(freq_filename);
  freq_file << "Frequency (Hz),Dither amplitude,Recording time (s)\n";
  for (size_t i = 0; i < dither_freqs.size(); i++) {
    freq_file << std::to_string(dither_freqs[i]) << "," << dither_amps[i] << "," << recording_times[i] << "\n";
  }
  freq_file.close();

  // reset all
  loop.reset_all();
  loop.p.store(P);
  loop.i.store(I);

  // OPEN LOOP CHARACTERISATION
  std::cout << "\t Open loop time series" << std::endl;
  // storage file
  std::string filename = dirname + "/open_loop.csv";

  characterise_open_loop(loop, sensorDataQueue, P, I, t_settle, t_record, filename);

  // CLOSED LOOP CHARACTERISATION
  std::cout << "\t Closed loop time series" << std::endl;
  // storage file
  filename = dirname + "/closed_loop.csv";
  std::ofstream file(filename);
  file << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
          "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

  // reset
  loop.reset_all();
  loop.p.store(P);
  loop.i.store(I);

  // turn on control loop
  loop.control_mode.store(4);

  // wait settling time
  std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

  // flush OPD queue
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

  // STEP RESPONSE CHARACTERISATION
  std::cout << "\t Step response" << std::endl;
  // storage file
  filename = dirname + "/step_response.csv";
  file.open(filename);
  file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

  // file 2
  std::string filename3 = dirname + "/step_response_sensor.csv";
  std::ofstream file3(filename3);
  file3 << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
           "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

  // reset control loop
  loop.reset_all();
  loop.p.store(P);
  loop.i.store(I);

  // settle control loop
  loop.control_mode.store(4);
  std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

  // flush data queues
  while (!loop.controlDataBuffer.isempty()) {
    loop.controlDataBuffer.pop();
  }
  while (!sensorDataQueue.isempty()) {
    sensorDataQueue.pop();
  }

  // record for 10 s: every 100 ms, toggle setpoint and record data
  t_start = std::chrono::high_resolution_clock::now();
  bool setpoint_high = false;
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
         10.0) {
    // wait 100 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // toggle setpoint
    if (setpoint_high) {
      loop.setpoint.store(0.0);
    } else {
      loop.setpoint.store(dither_amp);
    }
    setpoint_high = !setpoint_high;

    // write to file
    if (!loop.controlDataBuffer.isempty()) {
      int N = loop.controlDataBuffer.size();
      for (int i = 0; i < N; i++) {
        auto m = loop.controlDataBuffer.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
             << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
             << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
             << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
             << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
      }
    }

    // write to file 2
    if (!sensorDataQueue.isempty()) {
      int N = sensorDataQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = sensorDataQueue.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file3 << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
              << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
              << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
              << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
              << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
              << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
      }
    }
  }
  file.close();

  // CONTROL LAW FREQUENCY CHARACTERISATION
  std::cout << "\t Controller frequency characterisation" << std::endl;

  // make subdir: opd_controller_freq
  std::string subdir = dirname + "/freq_controller";
  std::filesystem::create_directory(subdir);

  // seperate storage file for each frequency
  for (size_t i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = subdir + "/" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

    // reset control loop
    loop.reset_all();
    loop.p.store(P);
    loop.i.store(I);

    // set dither loop parameters
    loop.dither_freq.store(dither_freqs[i]);
    loop.dither_amp.store(dither_amps[i]);

    // settle control loop
    loop.control_mode.store(2);
    std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

    // flush data queues
    while (!loop.controlDataBuffer.isempty()) {
      loop.controlDataBuffer.pop();
    }

    // record for recording time: every 10 ms, flush opd queue and write contents to file
    t_start = std::chrono::high_resolution_clock::now();
    while (
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
        recording_times[i]) {
      // wait 10 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      // write to file
      if (!loop.controlDataBuffer.isempty()) {
        int N = loop.controlDataBuffer.size();
        for (int i = 0; i < N; i++) {
          auto m = loop.controlDataBuffer.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
               << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
               << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
               << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
               << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
        }
      }
    }

    file.close();
  }

  // FREQUENCY CHARACTERISATION PLANT
  std::cout << "\t Plant frequency characterisation" << std::endl;

  // make subdir: opd_plant_freq
  std::string subdir1 = dirname + "/freq_plant";
  std::filesystem::create_directory(subdir1);

  // seperate storage file for each frequency

  for (size_t i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = subdir1 + "/control_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

    // second file: all sensor data
    std::string filename2 = subdir1 + "/sensor_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    std::ofstream file2(filename2);
    file2 << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
             "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

    // reset control loop
    loop.reset_all();
    loop.p.store(P);
    loop.i.store(I);

    // set dither loop parameters
    loop.dither_freq.store(dither_freqs[i]);
    loop.dither_amp.store(dither_amps[i]);

    // settle control loop
    loop.control_mode.store(1);
    std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

    // flush data queues
    while (!loop.controlDataBuffer.isempty()) {
      loop.controlDataBuffer.pop();
    }

    while (!sensorDataQueue.isempty()) {
      sensorDataQueue.pop();
    }

    // record for recording time: every 100 ms, flush opd queue and write contents to file.
    t_start = std::chrono::high_resolution_clock::now();
    while (
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
        recording_times[i]) {
      // wait 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write to file 1
      if (!loop.controlDataBuffer.isempty()) {
        int N = loop.controlDataBuffer.size();
        for (int i = 0; i < N; i++) {
          auto m = loop.controlDataBuffer.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
               << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
               << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
               << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
               << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
        }
      }

      // write to file 2
      if (!sensorDataQueue.isempty()) {
        int N = sensorDataQueue.size();
        for (int i = 0; i < N; i++) {
          auto m = sensorDataQueue.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file2 << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
                << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
                << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
                << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
                << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
                << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
        }
      }
    }
    file.close();
    file2.close();
  }

  // FREQUENCY CHARACTERISATION CLOSED LOOP DITHER PLANT
  // for all dither frequencies and amplitudes, run the control loop for a while and record dither and opd measurments
  std::cout << "\t Closed loop dither plant" << std::endl;

  std::string subdir2 = dirname + "/freq_closed_loop_dither_plant";
  std::filesystem::create_directory(subdir2);

  for (size_t i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = subdir2 + "/control_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

    // file2
    std::string filename2 = subdir2 + "/sensor_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    std::ofstream file2(filename2);
    file2 << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
             "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

    // reset control loop
    loop.reset_all();
    loop.p.store(P);
    loop.i.store(I);

    // set dither loop parameters
    loop.dither_freq.store(dither_freqs[i]);
    loop.dither_amp.store(dither_amps[i]);

    // settle control loop
    loop.control_mode.store(4);
    std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

    // flush data queues
    while (!loop.controlDataBuffer.isempty()) {
      loop.controlDataBuffer.pop();
    }

    while (!sensorDataQueue.isempty()) {
      sensorDataQueue.pop();
    }

    // record for recording time: every 100 ms, flush opd queue and write contents to file.
    t_start = std::chrono::high_resolution_clock::now();
    while (
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
        recording_times[i]) {
      // wait 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write to file
      if (!loop.controlDataBuffer.isempty()) {
        int N = loop.controlDataBuffer.size();
        for (int i = 0; i < N; i++) {
          auto m = loop.controlDataBuffer.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
               << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
               << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
               << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
               << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
        }
      }

      // write to file 2
      if (!sensorDataQueue.isempty()) {
        int N = sensorDataQueue.size();
        for (int i = 0; i < N; i++) {
          auto m = sensorDataQueue.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file2 << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
                << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
                << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
                << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
                << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
                << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
        }
      }
    }
    file.close();
    file2.close();
  }

  // FREQUENCY CHARACTERISATION CLOSED LOOP DITHER SETPOINT
  // for all dither frequencies and amplitudes, run the control loop for a while and record dither and opd measurments
  std::cout << "\t OPD frequency closed loop dither setpoint" << std::endl;

  std::string subdir3 = dirname + "/freq_closed_loop_dither_setpoint";
  std::filesystem::create_directory(subdir3);

  for (size_t i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = subdir3 + "/control_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

    // file2
    std::string filename2 = subdir3 + "/sensor_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    std::ofstream file2(filename2);
    file2 << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
             "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

    // reset control loop
    loop.reset_all();
    loop.p.store(P);
    loop.i.store(I);

    // set dither loop parameters
    loop.dither_freq.store(dither_freqs[i]);
    loop.dither_amp.store(dither_amps[i]);

    // settle control loop
    loop.control_mode.store(5);
    std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

    // flush data queues
    while (!loop.controlDataBuffer.isempty()) {
      loop.controlDataBuffer.pop();
    }

    while (!sensorDataQueue.isempty()) {
      sensorDataQueue.pop();
    }

    // record for recording time: every 100 ms, flush opd queue and write contents to file.
    t_start = std::chrono::high_resolution_clock::now();
    while (
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
        recording_times[i]) {
      // wait 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write to file
      if (!loop.controlDataBuffer.isempty()) {
        int N = loop.controlDataBuffer.size();
        for (int i = 0; i < N; i++) {
          auto m = loop.controlDataBuffer.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
               << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
               << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
               << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
               << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
        }
      }

      // write to file 2
      if (!sensorDataQueue.isempty()) {
        int N = sensorDataQueue.size();
        for (int i = 0; i < N; i++) {
          auto m = sensorDataQueue.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file2 << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
                << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
                << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
                << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
                << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
                << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
        }
      }
    }
    file.close();
    file2.close();
  }

  // DONE

  // reset control loop
  loop.reset_all();
  loop.p.store(P);
  loop.i.store(I);

  std::cout << "Finished control loop characterisation: " << description << std::endl;
};