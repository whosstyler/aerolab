// Settings: camera, simulation clock, debug overlays and rendering.

#include <array>
#include <format>
#include <string>

#include "editor/panels.h"
#include "editor/widgets.h"
#include "imgui.h"
#include "math/scalar.h"

namespace aerolab::editor {
namespace {

constexpr ImGuiTreeNodeFlags kOpen = ImGuiTreeNodeFlags_DefaultOpen;

bool PropertyFloat(const char* label, float* value, float speed, double min,
                   double max, const char* format,
                   const char* tooltip = nullptr) {
  double v = *value;
  if (!PropertyDouble(label, &v, speed, min, max, format, tooltip)) {
    return false;
  }
  *value = static_cast<float>(v);
  return true;
}

void DrawCameraSettings(CameraRig& rig) {
  if (!ImGui::CollapsingHeader("Camera", kOpen)) return;
  Camera& lens = rig.mutable_lens();
  CameraSettings& s = rig.settings();
  if (BeginProperties("##camera")) {
    double fov = Degrees(lens.vertical_fov_radians);
    if (PropertyDouble("Field of view", &fov, 0.2f, 10.0, 120.0, "%.0f deg")) {
      lens.vertical_fov_radians = Radians(fov);
    }
    PropertyDouble("Near plane", &lens.near_plane, 0.005f, 0.01, 10.0,
                   "%.2f m");
    PropertyDouble("Far plane", &lens.far_plane, 10.0f, 100.0, 20000.0,
                   "%.0f m");
    PropertyDouble("Fly speed", &s.move_speed, 0.1f, 0.1, 2000.0, "%.1f m/s",
                   "Free-fly cruise speed (mouse wheel adjusts it too)");
    PropertyDouble("Boost x", &s.boost_multiplier, 0.05f, 1.0, 20.0, "%.1f");
    double sensitivity = s.rotate_sensitivity * 1000.0;
    if (PropertyDouble("Look sensitivity", &sensitivity, 0.05f, 0.5, 20.0,
                       "%.1f mrad/px")) {
      s.rotate_sensitivity = sensitivity / 1000.0;
    }
    PropertyDouble("Smoothing", &s.move_sharpness, 0.1f, 0.0, 60.0, "%.1f /s",
                   "Velocity response rate; 0 = instant");
    PropertyDouble("Zoom step", &s.zoom_step, 0.005f, 0.01, 0.5, "%.2f");
    EndProperties();
  }
}

void DrawSimulationSettings(SimulationSystem& sim) {
  if (!ImGui::CollapsingHeader("Simulation clock", kOpen)) return;
  if (BeginProperties("##clock")) {
    PropertyLabel("Fixed step",
                  "Every simulation step uses exactly this dt. "
                  "Coarser steps make integrator errors "
                  "obvious.");
    const std::array<double, 5> steps = {1.0 / 30.0, 1.0 / 60.0, 1.0 / 120.0,
                                         1.0 / 240.0, 1.0 / 480.0};
    const std::string current =
        std::format("{:.2f} ms ({:.0f} Hz)", sim.fixed_step() * 1000.0,
                    1.0 / sim.fixed_step());
    if (ImGui::BeginCombo("##step", current.c_str())) {
      for (double step : steps) {
        const std::string label =
            std::format("{:.2f} ms ({:.0f} Hz)", step * 1000.0, 1.0 / step);
        if (ImGui::Selectable(label.c_str(), step == sim.fixed_step())) {
          sim.set_fixed_step(step);
        }
      }
      ImGui::EndCombo();
    }
    double scale = sim.time_scale();
    if (PropertyDouble("Time scale", &scale, 0.01f, 0.0, 16.0, "x%.2f")) {
      sim.set_time_scale(scale);
    }
    EndProperties();
  }
}

void DrawDebugSettings(DebugViewOptions& d, EditorState& state) {
  if (!ImGui::CollapsingHeader("Debug overlays", kOpen)) return;
  if (BeginProperties("##debug")) {
    PropertyBool("World axes", &d.world_axes);
    PropertyBool("Trails", &d.trails);
    PropertyBool("Velocity", &d.velocity_vectors, "Cyan arrows");
    PropertyDouble("  scale", &d.velocity_scale, 0.005f, 0.01, 5.0,
                   "%.2f m per m/s");
    PropertyBool("Acceleration", &d.acceleration_vectors, "Amber arrows");
    PropertyDouble("  scale", &d.acceleration_scale, 0.005f, 0.01, 5.0,
                   "%.2f m per m/s^2");
    PropertyBool("Force terms", &d.force_vectors,
                 "Per-term arrows on the selection: gravity (white), drag "
                 "(red), thrust (orange), lift (green), spring (pink), "
                 "central (purple)");
    PropertyBool("Local frames", &d.local_frames, "Body X/Y/Z axes");
    PropertyBool("Bounding boxes", &d.bounding_boxes);
    PropertyBool("Drop lines", &d.drop_lines,
                 "Vertical line to the ground: reads altitude at a glance");
    PropertyBool("Attractors", &d.attractors,
                 "Spring anchors and central bodies");
    PropertyBool("Prediction", &d.predicted_path,
                 "Look-ahead of the selected body");
    PropertyDouble("  horizon", &d.prediction_seconds, 0.1f, 0.5, 60.0,
                   "%.1f s");
    PropertyBool("Labels", &state.show_labels);
    PropertyBool("HUD", &state.show_hud);
    EndProperties();
  }
}

void DrawRenderSettings(RenderSettings& r) {
  if (!ImGui::CollapsingHeader("Rendering")) return;
  if (BeginProperties("##render")) {
    PropertyFloat("Sun azimuth", &r.sun_azimuth_degrees, 0.5f, 0.0, 360.0,
                  "%.0f deg");
    PropertyFloat("Sun elevation", &r.sun_elevation_degrees, 0.25f, -10.0, 90.0,
                  "%.0f deg");
    PropertyFloat("Sun intensity", &r.sun_intensity, 0.01f, 0.0, 4.0, "%.2f");
    PropertyColor("Sun colour", &r.sun_color);
    PropertyFloat("Ambient", &r.ambient_intensity, 0.01f, 0.0, 2.0, "%.2f");
    PropertyColor("Sky zenith", &r.sky_zenith);
    PropertyColor("Sky horizon", &r.sky_horizon);
    PropertyColor("Ground", &r.ground_color);
    PropertyColor("Ground haze", &r.ground_horizon);
    PropertyBool("Fog", &r.fog);
    PropertyFloat("  density", &r.fog_density, 0.00005f, 0.0, 0.02, "%.5f");
    PropertyBool("Ground plane", &r.show_ground);
    PropertyFloat("  opacity", &r.ground_opacity, 0.01f, 0.0, 1.0, "%.2f");
    PropertyBool("Grid", &r.show_grid);
    PropertyFloat("  cell size", &r.grid_cell_size, 0.05f, 0.05, 100.0,
                  "%.2f m");
    PropertyFloat("  fade distance", &r.grid_fade_distance, 1.0f, 10.0, 5000.0,
                  "%.0f m");
    PropertyBool("Blob shadows", &r.blob_shadows);
    PropertyBool("Wireframe", &r.wireframe);
    PropertyBool("Axis gizmo", &r.show_axis_gizmo);
    PropertyLabel("MSAA");
    const std::array<int, 4> samples = {0, 2, 4, 8};
    const std::string current =
        r.msaa_samples > 0 ? std::format("{}x", r.msaa_samples) : "Off";
    if (ImGui::BeginCombo("##msaa", current.c_str())) {
      for (int s : samples) {
        const std::string label = s > 0 ? std::format("{}x", s) : "Off";
        if (ImGui::Selectable(label.c_str(), s == r.msaa_samples)) {
          r.msaa_samples = s;
        }
      }
      ImGui::EndCombo();
    }
    EndProperties();
  }
}

}  // namespace

void DrawSettingsPanel(EditorFrame& frame, EditorState& state) {
  if (!ImGui::Begin(kSettingsWindow)) {
    ImGui::End();
    return;
  }
  DrawCameraSettings(*frame.camera);
  DrawSimulationSettings(*frame.simulation);
  DrawDebugSettings(*frame.debug_options, state);
  DrawRenderSettings(*frame.render_settings);
  ImGui::End();
}

}  // namespace aerolab::editor
