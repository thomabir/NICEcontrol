// Exercises the generic Tango device application (src/apps/TangoDeviceApp.hpp) against a ThorlabsChopper device.
// The first argument is the name of the device. The test starts the blade, thus use a simulated controller:
//   make test-chopper
#include <chrono>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

#include "apps/TangoDeviceApp.hpp"

static int failures = 0;

static void report(const std::string &what, bool ok, const std::string &detail = "") {
  std::printf("%s%s%s%s\n", ok ? "OK    " : "FAIL  ", what.c_str(), detail.empty() ? "" : "  [",
              detail.empty() ? "" : (detail + "]").c_str());
  if (!ok) failures++;
}

static const TangoAttributeState *find(const TangoDeviceState &state, const std::string &name) {
  for (const TangoAttributeState &a : state.attributes) {
    if (a.name == name) return &a;
  }
  return nullptr;
}

// Run the cycles of the core until the condition is true, or until the cycle limit.
static bool run_until(TangoDeviceApp &app, const TangoDeviceCommands &command, const std::function<bool()> &done) {
  for (int i = 0; i < 1000; i++) {
    app.sense();
    app.act(command);
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::printf("Usage: %s <device name>\n", argv[0]);
    return 2;
  }
  TangoDeviceState state;
  TangoDeviceApp app(argv[1], state);
  TangoDeviceCommands command;

  report("connected", run_until(app, command, [&] { return state.connected; }));
  if (!state.connected) return 1;

  const TangoAttributeState *frequency = find(state, "Frequency");
  const TangoAttributeState *blade = find(state, "Blade");
  const TangoAttributeState *enabled = find(state, "Enabled");
  const TangoAttributeState *device_state = find(state, "State");
  const TangoAttributeState *id = find(state, "Identification");
  report("the attribute list has Frequency, Blade, Enabled, State and Identification",
         frequency && blade && enabled && device_state && id);
  if (!(frequency && blade && enabled && device_state && id)) return 1;

  report("Frequency is a writable number in Hz",
         frequency->kind == TangoValueKind::Number && frequency->writable && frequency->unit == "Hz");
  report("Blade is an enum with the part numbers as labels",
         blade->kind == TangoValueKind::Enum && blade->enum_labels.size() > 6 && blade->enum_labels[6] == "MC1F10HP");
  report("Enabled is a boolean", enabled->kind == TangoValueKind::Boolean);
  report("Identification is read only text", id->kind == TangoValueKind::Text && !id->writable);
  report(
      "the commands are Start and Stop among others",
      std::find(state.device_commands.begin(), state.device_commands.end(), "Start") != state.device_commands.end() &&
          std::find(state.device_commands.begin(), state.device_commands.end(), "Stop") != state.device_commands.end());

  report("all attributes read", run_until(app, command, [&] {
           for (const TangoAttributeState &a : state.attributes)
             if (!a.valid) return false;
           return true;
         }));
  report("state reads STANDBY", find(state, "State")->text == "STANDBY", find(state, "State")->text);

  auto write = [&](const std::string &name, const std::string &value) {
    command.write_attribute = name;
    command.write_value = value;
    command.write_count++;
  };
  auto run = [&](const std::string &name) {
    command.device_command = name;
    command.device_command_count++;
  };

  write("Frequency", "750");
  report("a written Frequency reads back",
         run_until(app, command, [&] { return find(state, "Frequency")->number == 750.0; }));
  write("Blade", "6");
  report("a written Blade reads back", run_until(app, command, [&] { return find(state, "Blade")->number == 6.0; }));
  report("no error after the writes", state.last_error.empty(), state.last_error);

  write("HarmonicMultiplier", "99");
  run_until(app, command, [&] { return !state.last_error.empty(); });
  report("a value above the maximum gives an error", !state.last_error.empty(), state.last_error);
  write("Frequency", "abc");
  run_until(app, command, [&] { return state.last_error.find("not a valid value") != std::string::npos; });
  report("a value that is not a number gives an error", state.last_error.find("not a valid value") != std::string::npos,
         state.last_error);

  run("Start");
  report("Start gives state ON", run_until(app, command, [&] { return find(state, "State")->text == "ON"; }));
  report("Enabled reads true", run_until(app, command, [&] { return find(state, "Enabled")->number == 1.0; }));
  write("Blade", "1");
  run_until(app, command, [&] { return !state.last_error.empty(); });
  report("a refusal of the controller gives an error", !state.last_error.empty(), state.last_error);

  write("Enabled", "0");
  report("Enabled written false gives state STANDBY",
         run_until(app, command, [&] { return find(state, "State")->text == "STANDBY"; }));

  std::printf("%d failures\n", failures);
  return failures == 0 ? 0 : 1;
}
