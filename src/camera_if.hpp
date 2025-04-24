/*
Example of a client using the TANGO C++ api. Runs an intensity calibration routine
*/

#pragma once

#include <tango.h>

#include <array>
#include <cmath>
#include <ctime>
#include <numeric>
#include <vector>

// using namespace Tango;

// int event_id;
// const string phot_attr("PhotValue");

// variable to store attribute responses
//
// DeviceData num_frames;
// // Variable to store tango state
//
// string filename_prefix;
// vector<double> phot_value_arr;

// struct for image 100x100
// struct Image {
//   constexpr static int width = 100;
//   constexpr static int height = 100;
//   std::array<unsigned short, width * height> data;
// };

void init_camera() {
  Tango::DeviceProxy *cam_device = new Tango::DeviceProxy("detectors/flir/1");
  cam_device->ping();
  Tango::DevState state;
  Tango::DeviceAttribute att_reply;

  try {
    att_reply = cam_device->read_attribute("State");
    att_reply >> state;

    if (state != 1) {
      std::cout << "Camera is already running" << std::endl;
      return;
    }

    cam_device->command_inout("Initialise");

  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    exit(-1);
  }
}

int get_size() {
  Tango::DeviceAttribute att_reply;
  Tango::DevState state;

  try {
    // Connect and ping camera
    Tango::DeviceProxy *cam_device = new Tango::DeviceProxy("detectors/flir/1");
    cam_device->ping();
    att_reply = cam_device->read_attribute("State");
    att_reply >> state;

    if (state == 1) {
      throw std::invalid_argument("Camera off");
    }

    int size;

    att_reply = cam_device->read_attribute("Image");
    size = att_reply.get_dim_x();
    return size;
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    exit(-1);
  }
}

std::vector<unsigned short> get_image() {
  Tango::DeviceAttribute att_reply;
  Tango::DevState state;

  try {
    // Connect and ping camera
    Tango::DeviceProxy *cam_device = new Tango::DeviceProxy("detectors/flir/1");
    cam_device->ping();
    att_reply = cam_device->read_attribute("State");
    att_reply >> state;

    if (state == 1) {
      throw std::invalid_argument("Camera off");
    }

    att_reply = cam_device->read_attribute("Image");
    std::vector<unsigned short> image;

    att_reply >> image;

    return image;
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    exit(-1);
  }
}

//
//
//
//
//
//
//
//
//

// // Photometric update event callback class
// class PhotValEventCallBack : public Tango::CallBack {
//   void push_event(Tango::EventData *);
// };

// // Photometric update event function
// void PhotValEventCallBack::push_event(Tango::EventData *myevent) {
//   Tango::DevVarDoubleArray *phot_val;
//   try {
//     /*cout << "PhotValEventCallBack::push_event(): called attribute "
//          << myevent->attr_name
//          << " event "
//          << myevent->event
//          << " (err="
//          << myevent->err
//          << ")" << endl;*/
//     if (!myevent->err) {
//       // Extract value and store in phot val
//       *(myevent->attr_value) >> phot_val;
//       // Append to global vector
//       phot_value_arr.push_back((*phot_val)[0]);
//       delete phot_val;
//     }
//   } catch (...) {
//     cout << "PhotValEventCallBack::push_event(): could not extract data !\n";
//   }
// }

// int main(unsigned int argc, char **argv)

// {
//   // Declare event
//   PhotValEventCallBack *photval_callback = new PhotValEventCallBack;
//   DevULong num_frames_int = 1000;
//   num_frames << num_frames_int;

//   DeviceAttribute filename("Filename", "");

//   // Get date for filename
//   time_t t = time(0);  // get time now
//   struct tm *now = localtime(&t);

//   char buffer[80];
//   strftime(buffer, 80, "_%Y-%m-%d_", now);
//   auto date = std::string(buffer);

//   try

//   {
//     // DeviceProxy *device = new DeviceProxy("sys/database/2");
//     // device->ping();

//     // Connect and ping camera
//     DeviceProxy *cam_device = new DeviceProxy("detectors/flir/1");
//     cam_device->ping();

//     // Connect and ping shutter
//     DeviceProxy *shutter_device = new DeviceProxy("motor/shutter/1");
//     shutter_device->ping();
//     shutter_device->command_inout("init");

//     // Check Camera is in correct state
//     att_reply = cam_device->read_attribute("State");
//     att_reply >> state;
//     if (state != 0) {
//       throw std::invalid_argument("Camera either off or still running!");
//     }
//     phot_value_arr.clear();

//     // Subscribe to photometric update event
//     event_id = cam_device->subscribe_event("PhotValue", Tango::USER_EVENT, photval_callback);
//     cout << "event_client() id = " << event_id << endl;

//     cout << "Camera ready!" << endl;

//     // Check if shutter is ready
//     att_reply = shutter_device->read_attribute("State");
//     att_reply >> state;
//     if (state != 0) {
//       throw std::invalid_argument("Shutter not available");
//     }

//     // MOVE SHUTTER FOR BACKGROUND
//     shutter_device->command_inout("Blocking");

//     // Check if shutter is done
//     att_reply = shutter_device->read_attribute("State");
//     att_reply >> state;
//     if (state != 0) {
//       throw std::invalid_argument("Shutter still running");
//     }

//     cout << "Shutter moved to block" << endl;

//     // TAKE BACKGROUND
//     // Set background filename

//     filename_prefix = "Background" + date;
//     filename << filename_prefix;
//     cam_device->write_attribute(filename);

//     // Start camera

//     cout << "Starting Background" << endl;
//     cam_device->command_inout("CalcBackground", num_frames);

//     // Poll until background is done

//     int finished = 0;
//     while (finished == 0) {
//       att_reply = cam_device->read_attribute("State");
//       att_reply >> state;
//       if (state == 0) {
//         finished = 1;
//       }
//       sleep(1);
//     }

//     cout << "Background taken" << endl;

//     // MOVE SHUTTER FOR I1

//     shutter_device->command_inout("I1");

//     // Check if shutter is done
//     att_reply = shutter_device->read_attribute("State");
//     att_reply >> state;
//     if (state != 0) {
//       throw std::invalid_argument("Shutter still running");
//     }

//     cout << "Shutter moved to I1" << endl;

//     // TAKE I1 DATA
//     // Set I1 filename

//     filename_prefix = "I1" + date;
//     filename << filename_prefix;
//     cam_device->write_attribute(filename);

//     // Start camera

//     cout << "Starting I1" << endl;

//     cam_device->command_inout("RecordFrames", num_frames);

//     // Poll until I1 is done
//     finished = 0;
//     while (finished == 0) {
//       att_reply = cam_device->read_attribute("State");
//       att_reply >> state;
//       if (state == 0) {
//         finished = 1;
//       }
//       sleep(1);
//     }

//     cout << "Finished taking I1 data" << endl;

//     // GET MEAN OF I1

//     // for (auto i: phot_value_arr)
//     //     cout << i << ' ';
//     // cout << endl;

//     auto I1_mean =
//         std::accumulate(std::begin(phot_value_arr), std::end(phot_value_arr), 0.0) / std::size(phot_value_arr);
//     cout << "I1 mean is " << I1_mean << endl;

//     // Clear photometry for next beam
//     phot_value_arr.clear();

//     // MOVE SHUTTER FOR I2

//     shutter_device->command_inout("I2");

//     // Check if shutter is done
//     att_reply = shutter_device->read_attribute("State");
//     att_reply >> state;
//     if (state != 0) {
//       throw std::invalid_argument("Shutter still running");
//     }

//     cout << "Shutter moved to I2" << endl;

//     // Set I2 filename

//     filename_prefix = "I2" + date;
//     filename << filename_prefix;
//     cam_device->write_attribute(filename);

//     // Start camera

//     cout << "Starting I2" << endl;

//     cam_device->command_inout("RecordFrames", num_frames);

//     // Poll until I2 is done
//     finished = 0;
//     while (finished == 0) {
//       att_reply = cam_device->read_attribute("State");
//       att_reply >> state;
//       if (state == 0) {
//         finished = 1;
//       }
//       sleep(1);
//     }

//     cout << "Finished taking I2 data" << endl;

//     // for (auto i: phot_value_arr)
//     //     cout << i << ' ';
//     // cout << endl;

//     // Calculate I2 mean
//     auto I2_mean =
//         std::accumulate(std::begin(phot_value_arr), std::end(phot_value_arr), 0.0) / std::size(phot_value_arr);

//     cout << "I2 mean is " << I2_mean << endl;

//     // Clear photometry for next use
//     phot_value_arr.clear();

//     // Move back to nulling position

//     shutter_device->command_inout("Unobstructed");

//     // Check if shutter is done
//     att_reply = shutter_device->read_attribute("State");
//     att_reply >> state;
//     if (state != 0) {
//       throw std::invalid_argument("Shutter still running");
//     }

//     cout << "Shutter moved to unobstructed" << endl;

//     // Calculate final number, which is the denominator normalised to exposure time of 1ms

//     double denom;
//     double intTime;

//     // Get integration time
//     att_reply = cam_device->read_attribute("IntTime");
//     att_reply >> intTime;

//     cout << "Int Time " << intTime << "ms" << endl;

//     // Interference denominator
//     denom = I1_mean + I2_mean + 2 * sqrt(I1_mean * I2_mean);
//     denom = denom / intTime;

//     cout << "Final output " << denom << endl;

//     // Finish communication

//     cam_device->unsubscribe_event(event_id);

//   }

//   catch (DevFailed &e) {
//     Except::print_exception(e);
//     exit(-1);
//   }
// }
