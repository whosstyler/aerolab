#include "core/string_util.h"

#include <algorithm>
#include <cctype>
#include <string_view>

namespace aerolab {

bool ContainsIgnoringCase(std::string_view text, std::string_view needle) {
  if (needle.empty()) return true;
  const auto equal = [](char a, char b) {
    return std::tolower(static_cast<unsigned char>(a)) ==
           std::tolower(static_cast<unsigned char>(b));
  };
  return std::search(text.begin(), text.end(), needle.begin(), needle.end(),
                     equal) != text.end();
}

}  // namespace aerolab
