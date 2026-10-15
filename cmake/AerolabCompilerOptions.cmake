# Shared compiler settings for every first-party target.
#
# aerolab_warnings: warning flags only (used by tests, which link GoogleTest).
# aerolab_options:  warnings plus the project rules: no exceptions, no RTTI.

add_library(aerolab_warnings INTERFACE)
add_library(aerolab_options INTERFACE)
target_link_libraries(aerolab_options INTERFACE aerolab_warnings)
target_compile_features(aerolab_warnings INTERFACE cxx_std_20)

if(MSVC)
  target_compile_options(aerolab_warnings INTERFACE /W4 /permissive- /utf-8)
  target_compile_definitions(aerolab_warnings INTERFACE
    NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES)
  target_compile_options(aerolab_options INTERFACE /GR- /EHs-c-)
  target_compile_definitions(aerolab_options INTERFACE _HAS_EXCEPTIONS=0)
  if(AEROLAB_WARNINGS_AS_ERRORS)
    target_compile_options(aerolab_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(aerolab_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion
    -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual)
  target_compile_options(aerolab_options INTERFACE -fno-exceptions -fno-rtti)
  if(AEROLAB_WARNINGS_AS_ERRORS)
    target_compile_options(aerolab_warnings INTERFACE -Werror)
  endif()
endif()

# Optional sanitizers for first-party code, e.g. -DAEROLAB_SANITIZE=undefined
# or address,undefined (Clang/GCC only).
set(AEROLAB_SANITIZE "" CACHE STRING "Comma-separated -fsanitize= list")
if(AEROLAB_SANITIZE AND NOT MSVC)
  target_compile_options(aerolab_warnings INTERFACE
    -fsanitize=${AEROLAB_SANITIZE} -fno-sanitize-recover=all
    -fno-omit-frame-pointer)
  target_link_options(aerolab_warnings INTERFACE
    -fsanitize=${AEROLAB_SANITIZE})
endif()

# Produces self-contained executables on MinGW so they run without the
# toolchain's runtime DLLs on PATH. Sanitizer runtimes need dynamic linking.
function(aerolab_configure_executable target)
  if(MINGW AND NOT AEROLAB_SANITIZE)
    target_link_options(${target} PRIVATE -static)
  endif()
endfunction()
