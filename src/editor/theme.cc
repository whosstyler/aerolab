#include "editor/theme.h"

#include "imgui.h"

namespace aerolab::editor {
namespace {

ImVec4 Rgb(int r, int g, int b, float a = 1.0f) {
  return ImVec4(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
                static_cast<float>(b) / 255.0f, a);
}

}  // namespace

void ApplyTheme(float dpi_scale) {
  ImGuiStyle& style = ImGui::GetStyle();
  style = ImGuiStyle();

  style.WindowPadding = ImVec2(10, 8);
  style.FramePadding = ImVec2(8, 4);
  style.CellPadding = ImVec2(6, 3);
  style.ItemSpacing = ImVec2(8, 6);
  style.ItemInnerSpacing = ImVec2(6, 4);
  style.IndentSpacing = 14;
  style.ScrollbarSize = 12;
  style.GrabMinSize = 10;

  style.WindowBorderSize = 1;
  style.ChildBorderSize = 1;
  style.PopupBorderSize = 1;
  style.FrameBorderSize = 0;
  style.TabBorderSize = 0;

  style.WindowRounding = 4;
  style.ChildRounding = 4;
  style.FrameRounding = 3;
  style.PopupRounding = 4;
  style.ScrollbarRounding = 6;
  style.GrabRounding = 3;
  style.TabRounding = 3;
  style.WindowMenuButtonPosition = ImGuiDir_None;
  style.SeparatorTextBorderSize = 1;

  const ImVec4 bg = Rgb(24, 26, 29);
  const ImVec4 panel = Rgb(30, 32, 36);
  const ImVec4 raised = Rgb(40, 43, 48);
  const ImVec4 hover = Rgb(52, 56, 63);
  const ImVec4 active = Rgb(62, 67, 75);
  const ImVec4 border = Rgb(56, 60, 68, 0.9f);
  const ImVec4 text = Rgb(220, 223, 228);
  const ImVec4 text_dim = Rgb(140, 146, 156);
  const ImVec4 accent = Rgb(77, 143, 204);
  const ImVec4 accent_hover = Rgb(96, 160, 218);
  const ImVec4 accent_soft = Rgb(77, 143, 204, 0.35f);

  ImVec4* c = style.Colors;
  c[ImGuiCol_Text] = text;
  c[ImGuiCol_TextDisabled] = text_dim;
  c[ImGuiCol_WindowBg] = panel;
  c[ImGuiCol_ChildBg] = Rgb(0, 0, 0, 0.0f);
  c[ImGuiCol_PopupBg] = Rgb(34, 36, 41, 0.98f);
  c[ImGuiCol_Border] = border;
  c[ImGuiCol_BorderShadow] = Rgb(0, 0, 0, 0.0f);
  c[ImGuiCol_FrameBg] = raised;
  c[ImGuiCol_FrameBgHovered] = hover;
  c[ImGuiCol_FrameBgActive] = active;
  c[ImGuiCol_TitleBg] = bg;
  c[ImGuiCol_TitleBgActive] = bg;
  c[ImGuiCol_TitleBgCollapsed] = bg;
  c[ImGuiCol_MenuBarBg] = bg;
  c[ImGuiCol_ScrollbarBg] = Rgb(0, 0, 0, 0.0f);
  c[ImGuiCol_ScrollbarGrab] = raised;
  c[ImGuiCol_ScrollbarGrabHovered] = hover;
  c[ImGuiCol_ScrollbarGrabActive] = active;
  c[ImGuiCol_CheckMark] = accent;
  c[ImGuiCol_SliderGrab] = accent;
  c[ImGuiCol_SliderGrabActive] = accent_hover;
  c[ImGuiCol_Button] = raised;
  c[ImGuiCol_ButtonHovered] = hover;
  c[ImGuiCol_ButtonActive] = active;
  c[ImGuiCol_Header] = accent_soft;
  c[ImGuiCol_HeaderHovered] = Rgb(77, 143, 204, 0.25f);
  c[ImGuiCol_HeaderActive] = Rgb(77, 143, 204, 0.45f);
  c[ImGuiCol_Separator] = border;
  c[ImGuiCol_SeparatorHovered] = accent_soft;
  c[ImGuiCol_SeparatorActive] = accent;
  c[ImGuiCol_ResizeGrip] = Rgb(0, 0, 0, 0.0f);
  c[ImGuiCol_ResizeGripHovered] = accent_soft;
  c[ImGuiCol_ResizeGripActive] = accent;
  c[ImGuiCol_Tab] = bg;
  c[ImGuiCol_TabHovered] = hover;
  c[ImGuiCol_TabSelected] = panel;
  c[ImGuiCol_TabSelectedOverline] = accent;
  c[ImGuiCol_TabDimmed] = bg;
  c[ImGuiCol_TabDimmedSelected] = panel;
  c[ImGuiCol_TabDimmedSelectedOverline] = Rgb(0, 0, 0, 0.0f);
  c[ImGuiCol_DockingPreview] = accent_soft;
  c[ImGuiCol_DockingEmptyBg] = bg;
  c[ImGuiCol_PlotLines] = accent;
  c[ImGuiCol_PlotLinesHovered] = accent_hover;
  c[ImGuiCol_PlotHistogram] = accent;
  c[ImGuiCol_PlotHistogramHovered] = accent_hover;
  c[ImGuiCol_TableHeaderBg] = raised;
  c[ImGuiCol_TableBorderStrong] = border;
  c[ImGuiCol_TableBorderLight] = Rgb(48, 52, 58);
  c[ImGuiCol_TableRowBg] = Rgb(0, 0, 0, 0.0f);
  c[ImGuiCol_TableRowBgAlt] = Rgb(255, 255, 255, 0.025f);
  c[ImGuiCol_TextSelectedBg] = accent_soft;
  c[ImGuiCol_DragDropTarget] = accent;
  c[ImGuiCol_NavCursor] = accent;
  c[ImGuiCol_ModalWindowDimBg] = Rgb(0, 0, 0, 0.5f);

  style.ScaleAllSizes(dpi_scale);
  style.FontScaleDpi = dpi_scale;
}

}  // namespace aerolab::editor
