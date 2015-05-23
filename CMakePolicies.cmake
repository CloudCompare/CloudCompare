# Eliminate warnings with newer versions of CMake
# We use the OLD behavior by default
if (POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()
if (POLICY CMP0043)
  cmake_policy(SET CMP0043 OLD)
endif()
