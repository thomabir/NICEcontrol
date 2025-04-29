#include "NiceGui.hpp"          // GUI handling class
#include "SharedResources.hpp"  // Shared resources for data processing
#include "Workers.hpp"          // worker threads for data processing

int main() {
  SharedResources resources;
  Workers workers(resources);
  NiceGui gui;

  // Initialize GUI
  if (!gui.Initialize()) {
    std::cerr << "Failed to initialize GUI" << std::endl;
    return 1;
  }

  // Render loop
  while (!gui.ShouldClose()) {
    gui.StartFrame();
    gui.RenderFrame(resources, workers);
  }

  return 0;
}
