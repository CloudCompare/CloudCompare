CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
IF(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

PROJECT (GMMREG_LIB)

# The CMake command FIND_PACKAGE(VXL) attempts to find VXL binary
# installation.  CMake will look in the directory specified by the
# CMake variable VXL_DIR.  Normally, CMake will initially not find
# VXL, will warn you that it could not find VXL, and then give you the
# chance to set the variable VXL_DIR and reconfigure.  VXL_DIR now
# replaces VXL_BINARY_PATH.

FIND_PACKAGE(VXL)

# Whether FIND_PACKAGE(VXL) worked is stored in the variable
# VXL_FOUND.  If VXL was found we then include `UseVXL.cmake'
# which will set many variables prefixed with VXL_ that your project
# can use to determine what parts of VXL are present and how to use
# them.  `UseVXL.cmake' holds all the necessary definitions that
# CMake needs to use VXL.

IF(VXL_FOUND)
  INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)
ENDIF(VXL_FOUND)

# `UseVXL.cmake' will in turn include the file
# `VXLConfig.cmake' from the top of the VXL build tree.  You
# should look at this file to see what CMake variables are imported.

# To set VXL_DIR, you could rely on an environment variable:
# SET( VXL_DIR $ENV{VXLBIN} )
# However, you must then make sure that VXLBIN is set correctly every
# time that cmake is run.  This is often a source of much frustration
# and is not recommended.
#
# Another option is to set the path directly:
#   SET( VXL_DIR/usr/local/vxl-1.0-beta2/bin )
# This has the drawback that the CMakeLists.txt file (this file) has
# to be modified to suit your particular setup, which may cause
# trouble.  If this file is under CVS control, for example, you may
# inadvertently commit it with your local setup hard-coded, thus
# playing havoc with other users.

# The rest of this CMakeLists.txt file could contain commands as seen
# in the previous examples, or might just be SUBDIRS commands.


# Remember that all CMake variables defining the VXL configuration
# begin with "VXL_".  If you want to use VGUI in a client project you
# should use something like this.

SET(INI_LIBRARY_TYPE STATIC)
SET(API_LIBRARY_TYPE STATIC)

IF(WIN32)
  ADD_DEFINITIONS(-DWIN32 -D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE)
ELSE(WIN32)
  ADD_LIBRARY(port_ini ${INI_LIBRARY_TYPE} port_ini.c)
ENDIF(WIN32)

SET(GMMREG_API_SRCS
    gmmreg_api.cpp
    gmmreg_base.cpp
    gmmreg_cpd.cpp
    gmmreg_grbf.cpp
    gmmreg_grbf_func.cpp
    gmmreg_rigid.cpp
    gmmreg_rigid_func.cpp
    gmmreg_tps.cpp
    gmmreg_tps_func.cpp
    utils/misc_utils.cc
)
file(GLOB_RECURSE GMMREG_API_HDRS *.h)

INCLUDE_DIRECTORIES( ${VXL_VNL_INCLUDE_DIR} )

ADD_LIBRARY(${PROJECT_NAME} ${API_LIBRARY_TYPE} ${GMMREG_API_SRCS} ${GMMREG_API_HDRS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} vnl_algo vnl vcl)
IF(UNIX)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} m port_ini)
ENDIF(UNIX)
