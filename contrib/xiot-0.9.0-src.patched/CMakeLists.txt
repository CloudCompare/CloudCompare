cmake_minimum_required(VERSION 2.6)

project(xiot)
 
# Setup output paths.
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH
    "Single output directory for building all libraries.")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH
    "Single output directory for building all executables.")
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH EXECUTABLE_OUTPUT_PATH)

if (CMAKE_CL_64)
	SET(WIN_ARCH win64)
else (CMAKE_CL_64)
	if (WIN32)
	SET(WIN_ARCH win32)
	endif (WIN32)
endif (CMAKE_CL_64)

message(STATUS "Windows architecture is: ${WIN_ARCH}")

SET(CMAKE_MODULE_PATH "${xiot_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")


add_subdirectory(src)
add_subdirectory(tests)

#Documentation
option(BUILD_XIOT_DOCS "Build Doxygen documentation" OFF)
if(BUILD_XIOT_DOCS)
  include(TargetDoc)
  if (WIN32)
	install(FILES ./doc/README_SDK9 DESTINATION . RENAME README)
	if(EXISTS ${PROJECT_BINARY_DIR}/doc/html)
		install(DIRECTORY ${PROJECT_BINARY_DIR}/doc/html DESTINATION doc)
	endif(EXISTS ${PROJECT_BINARY_DIR}/doc/html)
  endif (WIN32)
endif(BUILD_XIOT_DOCS)

# Examples
option(BUILD_XIOT_EXAMPLES "Enable to build Examples (VTK loader)" OFF)
if(BUILD_XIOT_EXAMPLES)
  add_subdirectory(examples)
endif(BUILD_XIOT_EXAMPLES)

#ParaView plugin
option(BUILD_PARAVIEW_PLUGIN "Build ParaView X3D reader plugin" OFF)
if(BUILD_PARAVIEW_PLUGIN)
  add_subdirectory(plugins)
endif(BUILD_PARAVIEW_PLUGIN)

install(FILES README COPYING DESTINATION .)


#CPack
include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "9")
set(CPACK_PACKAGE_VERSION_PATCH "0")
if(WIN32 AND NOT UNIX)
  set(CPACK_GENERATOR "ZIP;NSIS")
  set(CPACK_SOURCE_GENERATOR "ZIP")
else(WIN32 AND NOT UNIX)
  set(CPACK_GENERATOR "STGZ;TGZ;")
  set(CPACK_SOURCE_GENERATOR "TGZ")
endif(WIN32 AND NOT UNIX)
set(CPACK_SOURCE_PACKAGE_FILE_NAME 
 "xiot-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}-src")
set(CPACK_SOURCE_IGNORE_FILES "/CVS/;/\\\\.svn/;.tex;\\\\.swp$;\\\\.#;/#;/_build/;/contrib/;/integration/;/data/")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${xiot_SOURCE_DIR}/README")
set(CPACK_RESOURCE_FILE_LICENSE "${xiot_SOURCE_DIR}/COPYING")



include(CPack)
