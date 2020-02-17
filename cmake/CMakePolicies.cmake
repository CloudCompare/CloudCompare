# Eliminate warnings with newer versions of CMake
if ( WIN32 )
	# https://cmake.org/cmake/help/v3.0/policy/CMP0020.html
    cmake_policy(SET CMP0020 NEW)
	# https://cmake.org/cmake/help/v3.0/policy/CMP0071.html
	cmake_policy(SET CMP0071 NEW)
endif()
