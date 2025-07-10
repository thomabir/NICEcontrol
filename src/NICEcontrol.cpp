#include "NiceGui.hpp"          // GUI handling class
#include "SharedResources.hpp"  // Shared resources for data processing
#include "Workers.hpp"          // worker threads for data processing

// initialise tip/tilt stages
char serial_number1[1024] = "0122040101";
PI_E727_Controller tip_tilt_stage1(serial_number1);

char serial_number2[1024] = "0122042007";
PI_E727_Controller tip_tilt_stage2(serial_number2);

nF_EBD_Controller nF_stage_1("/dev/ttyUSB3");
nF_EBD_Controller nF_stage_2("/dev/ttyUSB4");

void setupActuators() {
  // connect and intialise all piezo stages

  // Tip/tilt stage 1
  tip_tilt_stage1.init();
  // tip_tilt_stage1.autozero(); // run autozero if stage does not move
  tip_tilt_stage1.move_to_x(0.0f);
  tip_tilt_stage1.move_to_y(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "\tPI Stage 1 Position: (" << tip_tilt_stage1.readx() << ", " << tip_tilt_stage1.ready() << ") urad"
            << std::endl;

  // Tip/tilt stage 2
  tip_tilt_stage2.init();
  // tip_tilt_stage2.autozero(); // run autozero if stage does not move
  tip_tilt_stage2.move_to_x(0.0f);
  tip_tilt_stage2.move_to_y(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "\tPI Stage 2 Position: (" << tip_tilt_stage2.readx() << ", " << tip_tilt_stage2.ready() << ") urad"
            << std::endl;

  // nF tip/tilt stages
  nF_stage_1.init();
  nF_stage_1.move_to({0.0, 0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto pos = nF_stage_1.read();
  std::cout << "nF Stage 1 Position: " << pos[0] << ", " << pos[1] << std::endl;

  nF_stage_2.init();
  nF_stage_2.move_to({0.0, 0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pos = nF_stage_2.read();
  std::cout << "nF Stage 2 Position: " << pos[0] << ", " << pos[1] << std::endl;
}

int main() {
  SharedResources resources;
  Workers workers(resources);
  NiceGui gui;

  // Setup actuators
  // setupActuators();

  // Initialize GUI
  if (!gui.Initialize()) {
    std::cerr << "Failed to initialize GUI" << std::endl;
    return 1;
  }

  // Start some of the workers
  workers.metrology_reader.start();

  // Render loop
  while (!gui.ShouldClose()) {
    gui.StartFrame();
    gui.RenderFrame(resources, workers);
  }

  return 0;
}
