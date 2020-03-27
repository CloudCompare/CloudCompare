cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0071 NEW)
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

# --- Python wrapping with sip

INCLUDE_DIRECTORIES(
  ${SIP_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

SET(_add_SOURCES
    sipcloudCompareccPApi.cc
    sipcloudCompareccPointCloud.cc
    sipcloudCompareCCLib.cc
    sipcloudCompareCCLibScalarField.cc
)

MESSAGE(STATUS "_add_SOURCES: " "${_add_SOURCES}")

set_source_files_properties( ${_add_SOURCES} PROPERTIES GENERATED TRUE )

# --- sources / sip wrappings

SIP_WRAP_SIP( _sip_SOURCES cloudCompare.sip OPTIONS -t WS_X11 -t Qt_5_9_5 -I /usr/share/sip/PyQt5)
MESSAGE(STATUS "_sip_SOURCES: " "${_sip_SOURCES}")

# --- sources / to compile

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
