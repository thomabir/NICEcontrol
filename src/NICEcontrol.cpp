#include "Core.hpp"
#include "NiceGui.hpp"

int main() {
  Core core;

  // The user interface subscribes to the whiteboard streams before the core thread starts to write to them.
  NiceGui gui(core);

  core.start();
  gui.start();
  gui.wait_for_close();
  core.request_stop();

  return 0;
}
