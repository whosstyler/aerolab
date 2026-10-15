#ifndef AEROLAB_EDITOR_PANELS_H_
#define AEROLAB_EDITOR_PANELS_H_

#include <optional>

#include "editor/editor.h"
#include "scene/entity.h"
#include "world/presets.h"
#include "world/spawner.h"

namespace aerolab::editor {

// Window titles double as stable docking identifiers.
inline constexpr const char* kViewportWindow = "Viewport";
inline constexpr const char* kHierarchyWindow = "Hierarchy";
inline constexpr const char* kInspectorWindow = "Inspector";
inline constexpr const char* kSettingsWindow = "Settings";
inline constexpr const char* kTelemetryWindow = "Telemetry";
inline constexpr const char* kLogWindow = "Event Log";
inline constexpr const char* kPerformanceWindow = "Performance";

// Panels. Each draws one dockable window.
void DrawViewportPanel(EditorFrame& frame, const EditorState& state,
                       ViewportState& viewport);
void DrawHierarchyPanel(EditorFrame& frame, EditorState& state);
void DrawInspectorPanel(EditorFrame& frame);
void DrawTelemetryPanel(EditorFrame& frame, EditorState& state);
void DrawEventLogPanel(EditorFrame& frame, EditorState& state);
void DrawPerformancePanel(EditorFrame& frame, EditorState& state,
                          EditorRequests& requests);
void DrawSettingsPanel(EditorFrame& frame, EditorState& state);

// Multiplier for hard-coded pixel sizes (1 on a standard-DPI display).
float Dpi();

// Shared editor actions.

// Spawns an object a short distance in front of the camera (or at the
// orbit pivot), facing away from it, optionally with a motion preset, and
// selects it.
EntityId SpawnObject(EditorFrame& frame, Archetype archetype,
                     std::optional<MotionPreset> preset);

// Menu items for every archetype x motion preset combination.
void SpawnMenuItems(EditorFrame& frame);

// Resets the simulation and starts a new run in the event log, so the
// timeline only shows events from the current run.
void ResetSimulation(EditorFrame& frame);

void FocusSelection(EditorFrame& frame);
void FrameWholeScene(EditorFrame& frame);
void DeleteSelection(EditorFrame& frame);

}  // namespace aerolab::editor

#endif  // AEROLAB_EDITOR_PANELS_H_
