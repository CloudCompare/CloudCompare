
##########################################################################
#                                                                        #
#                               SIP                                      #
#                                                                        #
#  This program is free software; you can redistribute it and/or modify  #
#  it under the terms of the GNU Library General Public License as       #
#  published by the Free Software Foundation; version 2 or later of the  #
#  License.                                                              #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
#  GNU General Public License for more details.                          #
#                                                                        #
#          Copyright 2020 Paul RASCLE www.openfields.fr                  #
#                                                                        #
##########################################################################

message(STATUS "SIP processing")

# --- Python detection: if several versions installed, select one with all the required packages installed

set(PYTHON_PREFERED_VERSION "3.6" CACHE STRING "Select a Python version: it should be installed with all packages required for CloudCompare (pyqt, sip, numpy)")
set_property(CACHE PYTHON_PREFERED_VERSION PROPERTY STRINGS "3.6" "3.7" "3.8")
message(STATUS "python prefered version: " ${PYTHON_PREFERED_VERSION})
set(Python_ADDITIONAL_VERSIONS ${PYTHON_PREFERED_VERSION}) # priority to this version 
find_package(PythonInterp)
find_package(PythonLibs)
message(STATUS "python version: " ${PYTHON_VERSION_STRING})
message(STATUS "check match for ${PYTHON_PREFERED_VERSION}.x")
if(NOT ${PYTHON_VERSION_STRING} MATCHES "${PYTHON_PREFERED_VERSION}\\..")
    message(SEND_ERROR "no Python found matching ${PYTHON_PREFERED_VERSION}.x")
else()
    message(STATUS "Python found ${PYTHON_VERSION_STRING} matches prefered version ${PYTHON_PREFERED_VERSION}.x")
    message(STATUS "PYTHON_LIBRARIES:" ${PYTHON_LIBRARIES})
endif()

# --- PyQt Sip detection

find_package(PYQTSIP REQUIRED)  # should come after Python

# --- ----------------------------------------------------------
# --- target PyQt sip Python library: Python module cloudCompare

project( cloudCompare VERSION 0.1 )

# --- sources / sip wrappings

set(SIP_MAIN_SOURCE cloudCompare.sip)
set(SIP_ADD_SOURCES
    ccPApi.sip
    ccPointCloud.sip
    ScalarField.sip
    )
set(SIP_ALL_SOURCES ${SIP_MAIN_SOURCE} ${SIP_ADD_SOURCES})
message(STATUS "SIP_ALL_SOURCES: " "${SIP_ALL_SOURCES}")

# TODO: names of generated sip files follow some rules based on module name, class name, namespace etc. Not obvious to compute...

set(SIP_MAIN_GENSRC ${CMAKE_CURRENT_BINARY_DIR}/sipcloudComparecmodule.cc)
set(SIP_ADD_GENSRC
    ${CMAKE_CURRENT_BINARY_DIR}/sipcloudCompareccPointCloud.cc
    ${CMAKE_CURRENT_BINARY_DIR}/sipcloudCompareQList0101ccPointCloud.cc
    ${CMAKE_CURRENT_BINARY_DIR}/sipcloudCompareCCLib.cc
    ${CMAKE_CURRENT_BINARY_DIR}/sipcloudCompareCCLibScalarField.cc
   )
set(SIP_ALL_GENSRC ${SIP_MAIN_GENSRC} ${SIP_ADD_GENSRC})

message(STATUS "SIP_ALL_GENSRC: " "${SIP_ALL_GENSRC}")

set_source_files_properties( ${SIP_ALL_GENSRC} PROPERTIES GENERATED TRUE )

set(SIP_OPTIONS -s .cc -c . ${PYQT_SIPFLAGS})
message(STATUS "SIP_OPTIONS: " "${SIP_OPTIONS}")

add_custom_command(
    OUTPUT ${SIP_ALL_GENSRC}
    COMMAND ${SIP_EXECUTABLE} ${SIP_OPTIONS} ${CMAKE_CURRENT_SOURCE_DIR}/${SIP_MAIN_SOURCE}
    DEPENDS ${SIP_ALL_SOURCES}
    )

# --- sources / to compile

add_library( cloudCompare SHARED ${SIP_ALL_GENSRC} )

# --- Python wrapping with sip

target_include_directories( cloudCompare PUBLIC
    ${SIP_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

# --- Numpy includes

find_path(NUMPY_INCLUDE_DIR numpy/npy_common.h HINTS ${PYTHON_INCLUDE_DIRS} DOC "directory path of include: numpy/npy_common.h")
message(STATUS "NUMPY_INCLUDE_DIR: " ${NUMPY_INCLUDE_DIR})

target_include_directories( cloudCompare PUBLIC
    ${NUMPY_INCLUDE_DIR}
    )

target_link_libraries( cloudCompare
    ${PYTHON_LIBRARIES}
    CC_PAPI_LIB
    CC_FBO_LIB
    CC_CORE_LIB
    QCC_DB_LIB
    QCC_GL_LIB
    qcustomplot
    )

# Qt libraries

target_link_libraries( cloudCompare
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    Qt5::PrintSupport
    )

if (WIN32)
   target_link_libraries( cloudCompare Qt5::WinMain)
endif()

if( WIN32 OR APPLE)
   foreach( dest ${INSTALL_DESTINATIONS} )
       message(STATUS "INSTALL_DESTINATIONS: " ${dest})
       install_shared( cloudCompare ${dest} 1 )
   endforeach()
else()
    set(INSTALL_CLOUDCOMPARE_PYTHON ${CMAKE_INSTALL_LIBDIR}/cloudcompare)
    message(STATUS "INSTALL_CLOUDCOMPARE_PYTHON: " ${INSTALL_CLOUDCOMPARE_PYTHON})
    install(TARGETS cloudCompare LIBRARY DESTINATION ${INSTALL_CLOUDCOMPARE_PYTHON})
endif()

if(WIN32)
  set_target_properties(cloudCompare PROPERTIES SUFFIX ".pyd" DEBUG_OUTPUT_NAME cloudCompare_d RELEASE_OUTPUT_NAME cloudCompare)
else()
  set_target_properties(cloudCompare PROPERTIES PREFIX "")
endif()
