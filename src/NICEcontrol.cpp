#include "NiceGui.hpp"          // GUI handling class
#include "SharedResources.hpp"  // Shared resources for data processing
#include "Workers.hpp"          // worker threads for data processing

int main() {
  SharedResources resources;
  Workers workers(resources);
  NiceGui gui(resources, workers);

  // Start the worker threads
  workers.metrology_reader.start();
  workers.ethercat_reader.start();
  workers.beam_controller.start();

  // Start the GUI thread
  gui.start();

  // Wait for GUI to close
  gui.wait_for_close();

  // request the workers to stop
  workers.metrology_reader.request_stop();
  workers.ethercat_reader.request_stop();
  // workers.beam_controller.request_stop();

  return 0;
}
