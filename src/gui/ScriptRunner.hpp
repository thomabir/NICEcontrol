#pragma once

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// Runs a program in a new session, with its output in a file, and does not wait for it.
// The program continues when this object or the caller stops, because it has its own session and no pipe to the
// caller.
class ScriptRunner {
 public:
  explicit ScriptRunner(std::string output_path) : output_path(std::move(output_path)) {}

  // Start the program. Give back false if a program of this runner is still running or if the start failed.
  bool start(const std::vector<std::string> &args) {
    if (running() || args.empty()) {
      return false;
    }
    std::vector<char *> argv;
    for (const std::string &a : args) {
      argv.push_back(const_cast<char *>(a.c_str()));
    }
    argv.push_back(nullptr);

    const pid_t child = fork();
    if (child < 0) {
      return false;
    }
    if (child == 0) {
      // The program must not keep the files and sockets of the caller open.
      setsid();
      const int out = open(output_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0664);
      const int in = open("/dev/null", O_RDONLY);
      dup2(in, STDIN_FILENO);
      dup2(out, STDOUT_FILENO);
      dup2(out, STDERR_FILENO);
      close_range(3, ~0U, 0);
      execv(argv[0], argv.data());
      _exit(127);
    }
    pid = child;
    exit_status = -1;
    ran = true;
    text.clear();
    return true;
  }

  // Read the output and look for the end of the program, without a wait.
  void poll() {
    if (pid <= 0) {
      return;
    }
    int status = 0;
    const bool ended = waitpid(pid, &status, WNOHANG) == pid;
    std::ifstream file(output_path);
    std::stringstream content;
    content << file.rdbuf();
    text = content.str();
    if (ended) {
      exit_status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      pid = -1;
    }
  }

  bool running() const { return pid > 0; }
  bool started() const { return ran; }

  // The exit status of the last program that ended, or -1 if it did not exit normally.
  int status() const { return exit_status; }

  // The output of the last program, as at the last poll.
  const std::string &output() const { return text; }

 private:
  std::string output_path;
  pid_t pid = -1;
  int exit_status = -1;
  bool ran = false;
  std::string text;
};
