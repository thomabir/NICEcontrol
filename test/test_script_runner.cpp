// Checks src/gui/ScriptRunner.hpp with short shell programs.
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#include "gui/ScriptRunner.hpp"

static int failures = 0;

static void expect(const std::string &what, bool ok) {
  std::printf("%-52s %s\n", what.c_str(), ok ? "OK" : "FAIL");
  if (!ok) failures++;
}

static void wait_for_end(ScriptRunner &runner) {
  for (int i = 0; i < 500 && runner.running(); i++) {
    runner.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main() {
  const std::string path = "/tmp/test_script_runner_" + std::to_string(getpid()) + ".log";
  ScriptRunner runner(path);
  expect("no program before the first start", !runner.started() && !runner.running());

  expect("the program starts", runner.start({"/bin/sh", "-c", "echo one; sleep 0.2; echo two >&2; exit 3"}));
  expect("a second start is refused while the program runs", !runner.start({"/bin/true"}));
  wait_for_end(runner);
  expect("the program ends", !runner.running() && runner.started());
  expect("the exit status is that of the program", runner.status() == 3);
  expect("the output has stdout and stderr", runner.output() == "one\ntwo\n");

  // The open files of the caller do not go to the program.
  const int fd = open("/dev/null", O_RDONLY);
  runner.start({"/bin/sh", "-c", "ls /proc/self/fd | wc -l"});
  wait_for_end(runner);
  expect("the program has only stdin, stdout, stderr", runner.output() == "4\n");  // and the fd of ls itself
  close(fd);

  // The program has its own session.
  runner.start({"/bin/sh", "-c", "ps -o sid= -p $$ | tr -d ' '; echo $$"});
  wait_for_end(runner);
  const std::string out = runner.output();
  const size_t newline = out.find('\n');
  expect("the program is the leader of a new session",
         newline != std::string::npos && out.substr(0, newline) == out.substr(newline + 1, out.size() - newline - 2));

  expect("a missing program gives exit status 127", runner.start({"/no/such/program"}));
  wait_for_end(runner);
  expect("and the runner reports it", runner.status() == 127);

  std::remove(path.c_str());
  if (failures == 0) std::printf("all checks passed\n");
  return failures == 0 ? 0 : 1;
}
