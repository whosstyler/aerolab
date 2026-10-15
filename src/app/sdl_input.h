#ifndef AEROLAB_APP_SDL_INPUT_H_
#define AEROLAB_APP_SDL_INPUT_H_

#include <SDL3/SDL_events.h>

#include "core/input.h"

namespace aerolab {

// Translates one SDL event into the engine's platform-independent input
// state. `pixel_density` converts SDL's window coordinates (points) into
// framebuffer pixels. Unhandled events are ignored.
void ApplySdlEvent(const SDL_Event& event, float pixel_density,
                   InputState& input);

}  // namespace aerolab

#endif  // AEROLAB_APP_SDL_INPUT_H_
