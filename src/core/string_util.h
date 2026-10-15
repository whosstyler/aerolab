#ifndef AEROLAB_CORE_STRING_UTIL_H_
#define AEROLAB_CORE_STRING_UTIL_H_

#include <string_view>

namespace aerolab {

// True if `needle` occurs in `text`, comparing ASCII letters without regard
// to case. An empty needle matches everything (used for UI filters).
bool ContainsIgnoringCase(std::string_view text, std::string_view needle);

}  // namespace aerolab

#endif  // AEROLAB_CORE_STRING_UTIL_H_
