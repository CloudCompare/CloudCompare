##########################################################################
#                                                                        #
#                               CCPAPI                                   #
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

cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0071 NEW)
MESSAGE(STATUS "CC PYthon API")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

# --- Python detection: if several versions installed, select one with all the required packages installed

SET(PYTHON_PREFERED_VERSION "3.6" CACHE STRING "Select a Python version: it should be installed with all packages required for CloudCompare (pyqt, sip, numpy)")
set_property(CACHE PYTHON_PREFERED_VERSION PROPERTY STRINGS "3.6" "3.7" "3.8")
MESSAGE(STATUS "python prefered version: " ${PYTHON_PREFERED_VERSION})
SET(Python_ADDITIONAL_VERSIONS ${PYTHON_PREFERED_VERSION}) # priority to this version 
FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
MESSAGE(STATUS "python version: " ${PYTHON_VERSION_STRING})
MESSAGE(STATUS "check match for ${PYTHON_PREFERED_VERSION}.x")
IF(NOT ${PYTHON_VERSION_STRING} MATCHES "${PYTHON_PREFERED_VERSION}\\..")
    MESSAGE(SEND_ERROR "no Python found matching ${PYTHON_PREFERED_VERSION}.x")
ELSE()
    MESSAGE(STATUS "Python found ${PYTHON_VERSION_STRING} matches prefered version ${PYTHON_PREFERED_VERSION}.x")
    MESSAGE(STATUS "PYTHON_LIBRARIES:" ${PYTHON_LIBRARIES})
ENDIF()

# --- Sip detection

FIND_PACKAGE(SIP REQUIRED)  # should come after Python (and before PyQt if needed)

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
include_directories( ${CloudCompare_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../common )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/db_tree )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/ui_templates )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../libs/qcustomplot )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

add_library( CC_PAPI_LIB STATIC ${header_list} ${source_list} )

target_link_libraries( CC_PAPI_LIB
    CC_FBO_LIB
    CC_CORE_LIB
    QCC_DB_LIB
    QCC_IO_LIB
    QCC_GL_LIB
    qcustomplot
    )

# --- Python wrapping with sip

INCLUDE_DIRECTORIES(
  ${SIP_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

# --- sources / sip wrappings

SET(SIP_MAIN_SOURCE cloudCompare.sip)
SET(SIP_ADD_SOURCES
    ccPApi.sip
    ccPointCloud.sip
    ScalarField.sip
    )
SET(SIP_ALL_SOURCES ${SIP_MAIN_SOURCE} ${SIP_ADD_SOURCES})

MESSAGE(STATUS "SIP_ALL_SOURCES: " "${SIP_ALL_SOURCES}")

# TODO: names of generated sip files follow some rules based on module name, class name, namespace etc. Not obvious to compute...

SET(SIP_MAIN_GENSRC ${CMAKE_CURRENT_BINARY_DIR}/sipcloudComparecmodule.cc)
SET(SIP_ADD_GENSRC
    ${CMAKE_CURRENT_BINARY_DIR}/sipcloudCompareccPointCloud.cc
    ${CMAKE_CURRENT_BINARY_DIR}/sipcloudCompareQList0101ccPointCloud.cc
    ${CMAKE_CURRENT_BINARY_DIR}/sipcloudCompareCCLib.cc
    ${CMAKE_CURRENT_BINARY_DIR}/sipcloudCompareCCLibScalarField.cc
   )
SET(SIP_ALL_GENSRC ${SIP_MAIN_GENSRC} ${SIP_ADD_GENSRC})

MESSAGE(STATUS "SIP_ALL_GENSRC: " "${SIP_ALL_GENSRC}")

set_source_files_properties( ${SIP_ALL_GENSRC} PROPERTIES GENERATED TRUE )

# TODO: generate the sip options (Qt version, sip path...)

SET(SIP_OPTIONS -s .cc -c . -t WS_X11 -t Qt_5_9_5 -I /usr/share/sip/PyQt5)

ADD_CUSTOM_COMMAND(
    OUTPUT ${SIP_ALL_GENSRC}
    COMMAND ${SIP_EXECUTABLE} ${SIP_OPTIONS} ${CMAKE_CURRENT_SOURCE_DIR}/${SIP_MAIN_SOURCE}
    DEPENDS ${SIP_ALL_SOURCES}
    )

# --- sources / to compile

ADD_LIBRARY( cloudCompare SHARED ${SIP_ALL_GENSRC} )

TARGET_LINK_LIBRARIES(cloudCompare
    ${PYTHON_LIBRARIES}
    CC_PAPI_LIB
    CC_FBO_LIB
    CC_CORE_LIB
    QCC_DB_LIB
    QCC_GL_LIB
    qcustomplot
    )

# Qt libraries

TARGET_LINK_LIBRARIES(cloudCompare
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    Qt5::PrintSupport
    )

IF (WIN32)
   TARGET_LINK_LIBRARIES(cloudCompare Qt5::WinMain)
ENDIF()

IF( WIN32 OR APPLE)
   FOREACH( dest ${INSTALL_DESTINATIONS} )
       MESSAGE(STATUS "INSTALL_DESTINATIONS: " ${dest})
       INSTALL_SHARED( cloudCompare ${dest} 1 )
   ENDFOREACH()
ELSE()
    SET(INSTALL_CLOUDCOMPARE_PYTHON ${CMAKE_INSTALL_LIBDIR}/cloudcompare)
    MESSAGE(STATUS "INSTALL_CLOUDCOMPARE_PYTHON: " ${INSTALL_CLOUDCOMPARE_PYTHON})
    INSTALL(TARGETS cloudCompare LIBRARY DESTINATION ${INSTALL_CLOUDCOMPARE_PYTHON})
ENDIF()

IF(WIN32)
  SET_TARGET_PROPERTIES(cloudCompare PROPERTIES SUFFIX ".pyd" DEBUG_OUTPUT_NAME cloudCompare_d RELEASE_OUTPUT_NAME cloudCompare)
ELSE()
  SET_TARGET_PROPERTIES(cloudCompare PROPERTIES PREFIX "")
ENDIF()
