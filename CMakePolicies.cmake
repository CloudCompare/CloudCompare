# Eliminate warnings with newer versions of CMake
# We use the OLD behavior by default

# https://cmake.org/cmake/help/v3.0/policy/CMP0020.html
if ( WIN32 )
    if (POLICY CMP0020)
        cmake_policy(SET CMP0020 OLD)
    endif()
endif()

# https://cmake.org/cmake/help/v3.0/policy/CMP0043.html
if (POLICY CMP0043)
    cmake_policy(SET CMP0043 OLD)
endif()
