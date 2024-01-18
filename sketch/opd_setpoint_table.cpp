// Adjusting OPD setpoint with buttons in a table
if (ImGui::BeginTable("table1", 6, ImGuiTableFlags_SizingFixedFit)) {
  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::Text("Increase:");
  ImGui::TableNextColumn();
  if (ImGui::Button("+0.1 nm")) {
    opd_setpoint_gui += 0.0001;
  }
  ImGui::TableNextColumn();
  if (ImGui::Button("+1 nm")) {
    opd_setpoint_gui += 0.001;
  }
  ImGui::TableNextColumn();
  if (ImGui::Button("+10 nm")) {
    opd_setpoint_gui += 0.01;
  }
  ImGui::TableNextColumn();
  if (ImGui::Button("+100 nm")) {
    opd_setpoint_gui += 0.1;
  }
  ImGui::TableNextColumn();
  if (ImGui::Button("+1 µm")) {
    opd_setpoint_gui += 1.;
  }

  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::Text("Decrease:");
  ImGui::TableNextColumn();
  if (ImGui::Button("-0.1 nm")) {
    opd_setpoint_gui -= 0.0001;
  }
  ImGui::TableNextColumn();
  if (ImGui::Button("-1 nm")) {
    opd_setpoint_gui -= 0.001;
  }
  ImGui::TableNextColumn();
  if (ImGui::Button("-10 nm")) {
    opd_setpoint_gui -= 0.01;
  }
  ImGui::TableNextColumn();
  if (ImGui::Button("-100 nm")) {
    opd_setpoint_gui -= 0.1;
  }
  ImGui::TableNextColumn();
  if (ImGui::Button("-1 µm")) {
    opd_setpoint_gui -= 1.;
  }

  ImGui::EndTable();
}