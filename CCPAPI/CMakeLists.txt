cmake_minimum_required(VERSION 3.0)

MESSAGE(STATUS "CC PYthon API")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

# --- Python detection

SET(Python_ADDITIONAL_VERSIONS 3.6) # priority to this version 
FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
MESSAGE(STATUS "python version: " ${PYTHON_VERSION_STRING})

# --- Sip detection

FIND_PACKAGE(SIP REQUIRED)  # should come after Python (and before PyQt if needed)
INCLUDE(UseSIP)

# --- Swig detection

# FIND_PACKAGE(SWIG REQUIRED)
# MESSAGE(STATUS "SWIG_USE_FILE " ${SWIG_USE_FILE})
# INCLUDE(${SWIG_USE_FILE})

# --- C++ library cc Python Api (ccPApi)

include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )
include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${QCC_DB_LIB_SOURCE_DIR} )
if( MSVC )
   include_directories( ${QCC_DB_LIB_SOURCE_DIR}/msvc )
endif()
include_directories( ${QCC_IO_LIB_SOURCE_DIR} )
include_directories( ${QCC_GL_LIB_SOURCE_DIR} )
include_directories( ${CloudComparePlugins_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
#include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../qCC )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../common )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/db_tree )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/ui_templates )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../libs/qcustomplot )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

add_library( CC_PAPI_LIB STATIC ${header_list} ${source_list} )

target_link_libraries( CC_PAPI_LIB CC_FBO_LIB )
target_link_libraries( CC_PAPI_LIB CC_CORE_LIB )
target_link_libraries( CC_PAPI_LIB QCC_DB_LIB )
target_link_libraries( CC_PAPI_LIB QCC_IO_LIB )
target_link_libraries( CC_PAPI_LIB QCC_GL_LIB )
target_link_libraries( CC_PAPI_LIB qcustomplot )

# Qt
#target_link_libraries(CC_PAPI_LIB Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL Qt5::PrintSupport)

# --- Python wrapping with sip

INCLUDE_DIRECTORIES(
  ${SIP_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

#SET(_link_LIBRARIES
#  ${PYTHON_LIBRARIES} " CC_PAPI_LIB CC_FBO_LIB CC_CORE_LIB QCC_DB_LIB QCC_GL_LIB qcustomplot ")
#MESSAGE(STATUS "_link_LIBRARIES: " ${_link_LIBRARIES})

SET(_add_SOURCES
    sipcloudCompareccPApi.cc
    sipcloudCompareccPointCloud.cc
)

set_source_files_properties( ${_add_SOURCES} PROPERTIES GENERATED TRUE )

# sources / sip wrappings
SIP_WRAP_SIP( _sip_SOURCES cloudCompare.sip OPTIONS -t WS_X11 -t Qt_5_9_5 -I /usr/share/sip/PyQt5)
MESSAGE(STATUS "_sip_SOURCES: " ${_sip_SOURCES})

# sources / to compile
ADD_LIBRARY( cloudCompare SHARED ${_sip_SOURCES} ${_add_SOURCES} )

TARGET_LINK_LIBRARIES(cloudCompare ${PYTHON_LIBRARIES} CC_PAPI_LIB CC_FBO_LIB CC_CORE_LIB QCC_DB_LIB QCC_GL_LIB qcustomplot Qt5::Core Qt5::Widgets Qt5::Concurrent)

SET(INSTALL_CLOUDCOMPARE_PYTHON ${CMAKE_INSTALL_LIBDIR}/cloudcompare)
MESSAGE(STATUS "INSTALL_CLOUDCOMPARE_PYTHON: " ${INSTALL_CLOUDCOMPARE_PYTHON})
INSTALL(TARGETS cloudCompare LIBRARY DESTINATION ${INSTALL_CLOUDCOMPARE_PYTHON})

IF(WIN32)
  SET_TARGET_PROPERTIES(cloudCompare PROPERTIES SUFFIX ".pyd" DEBUG_OUTPUT_NAME cloudCompare_d RELEASE_OUTPUT_NAME cloudCompare)
ELSE()
  SET_TARGET_PROPERTIES(cloudCompare PROPERTIES PREFIX "")
ENDIF()

# --- Python wrapping with swig

#INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
# SET(CMAKE_SWIG_FLAGS "")
# SET(_swig_SCRIPTS
#   ${CMAKE_CURRENT_BINARY_DIR}/cloudCompare.py
# )
# SET_SOURCE_FILES_PROPERTIES(cloudCompare.i PROPERTIES CPLUSPLUS ON)
# SWIG_ADD_LIBRARY(cloudCompare LANGUAGE python SOURCES cloudCompare.i)
# SWIG_LINK_LIBRARIES(cloudCompare CC_PAPI_LIB)
# SWIG_LINK_LIBRARIES(cloudCompare ${PYTHON_LIBRARIES})
# SET(INSTALL_CLOUDCOMPARE_PYTHON ${CMAKE_INSTALL_LIBDIR}/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages/cloudCompare)
# MESSAGE(STATUS "INSTALL_CLOUDCOMPARE_PYTHON: " ${INSTALL_CLOUDCOMPARE_PYTHON})
# INSTALL(TARGETS ${SWIG_MODULE_cloudCompare_REAL_NAME} LIBRARY DESTINATION ${INSTALL_CLOUDCOMPARE_PYTHON})
# SET(PERMS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
# INSTALL(FILES ${_swig_SCRIPTS} DESTINATION ${INSTALL_CLOUDCOMPARE_PYTHON} PERMISSIONS ${PERMS})
