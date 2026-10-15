// Checks src/gui/TangoDeviceWidgets.hpp in an ImGui context with no window and no renderer.
// The ImGui assertions are active, thus a widget call that ImGui does not accept stops this test.
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "gui/TangoDeviceWidgets.hpp"

static int failures = 0;

static void expect(const std::string &what, bool ok) {
  std::printf("%-52s %s\n", what.c_str(), ok ? "OK" : "FAIL");
  if (!ok) failures++;
}

static TangoAttributeState attribute(const std::string &name, TangoValueKind kind) {
  TangoAttributeState a;
  a.name = name;
  a.kind = kind;
  a.writable = true;
  a.valid = true;
  return a;
}

// One frame with an input for each attribute. Before draws the content in front of the inputs.
static void frame(std::vector<TangoAttributeState> &attributes, TangoEditBuffers &buffers,
                  std::vector<std::string> &sent, const std::function<void(size_t)> &before = nullptr) {
  ImGuiIO &io = ImGui::GetIO();
  io.DeltaTime = 1.0f / 60.0f;
  ImGui::NewFrame();
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(400, 400));
  ImGui::Begin("Test");
  for (size_t i = 0; i < attributes.size(); i++) {
    if (before) before(i);
    TangoAttributeInput("Test", attributes[i], buffers,
                        [&](const std::string &value) { sent.push_back(attributes[i].name + "=" + value); });
  }
  ImGui::End();
  ImGui::Render();
}

int main() {
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(800, 600);
  io.IniFilename = nullptr;
  unsigned char *pixels;
  int width, height;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

  std::vector<TangoAttributeState> attributes{
      attribute("Frequency", TangoValueKind::Number), attribute("Count", TangoValueKind::Integer),
      attribute("Enabled", TangoValueKind::Boolean), attribute("Blade", TangoValueKind::Enum),
      attribute("Name", TangoValueKind::Text)};
  attributes[0].number = 1000.0;
  attributes[3].enum_labels = {"A", "B"};
  attributes[4].text = "old";
  TangoEditBuffers buffers;
  std::vector<std::string> sent;

  frame(attributes, buffers, sent);
  expect("each kind of input draws", true);
  expect("a draw alone sends nothing", sent.empty());

  // Type into the Frequency input and press Enter.
  frame(attributes, buffers, sent, [](size_t i) {
    if (i == 0) ImGui::SetKeyboardFocusHere();
  });
  frame(attributes, buffers, sent);
  io.AddInputCharactersUTF8("750");
  frame(attributes, buffers, sent);
  expect("the input keeps the typed value during the edit", buffers.numbers["Test/Frequency"] == 750.0);
  expect("an edit in progress sends nothing", sent.empty());
  io.AddKeyEvent(ImGuiKey_Enter, true);
  frame(attributes, buffers, sent);
  io.AddKeyEvent(ImGuiKey_Enter, false);
  frame(attributes, buffers, sent);
  expect("Enter sends the typed value", sent.size() == 1 && sent[0] == "Frequency=750");
  frame(attributes, buffers, sent);
  expect("after the edit the input follows the read value", buffers.numbers["Test/Frequency"] == 1000.0);

  // Type into the Name input and press Enter.
  sent.clear();
  frame(attributes, buffers, sent, [](size_t i) {
    if (i == 4) ImGui::SetKeyboardFocusHere();
  });
  frame(attributes, buffers, sent);
  io.AddInputCharactersUTF8("new");
  frame(attributes, buffers, sent);
  io.AddKeyEvent(ImGuiKey_Enter, true);
  frame(attributes, buffers, sent);
  io.AddKeyEvent(ImGuiKey_Enter, false);
  frame(attributes, buffers, sent);
  expect("Enter sends the typed text", sent.size() == 1 && sent[0] == "Name=new");

  expect("an enum shows its label", TangoValueText(attributes[3]) == "A");
  attributes[3].number = 5.0;
  expect("an enum index with no label shows the index", TangoValueText(attributes[3]) == "5");

  ImGui::DestroyContext();
  if (failures == 0) std::printf("all checks passed\n");
  return failures == 0 ? 0 : 1;
}
