cmake_minimum_required(VERSION 3.0)

#project( cloudCompare )
MESSAGE(STATUS "CC PYthon API")

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

# Qt
#target_link_libraries(CC_PAPI_LIB Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL Qt5::PrintSupport)

FIND_PACKAGE(SWIG REQUIRED)
MESSAGE(STATUS "SWIG_USE_FILE " ${SWIG_USE_FILE})
INCLUDE(${SWIG_USE_FILE})

SET(Python_ADDITIONAL_VERSIONS 3.6) # priority to this version 
FIND_PACKAGE(PythonLibs)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(CMAKE_SWIG_FLAGS "")

SET(_swig_SCRIPTS
  ${CMAKE_CURRENT_BINARY_DIR}/cloudCompare.py
)

SET_SOURCE_FILES_PROPERTIES(cloudCompare.i PROPERTIES CPLUSPLUS ON)
#SET_SOURCE_FILES_PROPERTIES(cloudCompare.i PROPERTIES SWIG_FLAGS "-includeall")
SWIG_ADD_LIBRARY(cloudCompare LANGUAGE python SOURCES cloudCompare.i)
SWIG_LINK_LIBRARIES(cloudCompare CC_PAPI_LIB)
SWIG_LINK_LIBRARIES(cloudCompare ${PYTHON_LIBRARIES})
#install_shared( cloudCompare ${CMAKE_INSTALL_LIBDIR}/cloudcompare 0)
SET(INSTALL_CLOUDCOMPARE_PYTHON ${CMAKE_INSTALL_LIBDIR}/python3.6/site-packages/cloudCompare)
MESSAGE(STATUS "INSTALL_CLOUDCOMPARE_PYTHON " ${INSTALL_CLOUDCOMPARE_PYTHON})
INSTALL(TARGETS ${SWIG_MODULE_cloudCompare_REAL_NAME} LIBRARY DESTINATION ${INSTALL_CLOUDCOMPARE_PYTHON})
SET(PERMS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
INSTALL(FILES ${_swig_SCRIPTS} DESTINATION ${INSTALL_CLOUDCOMPARE_PYTHON} PERMISSIONS ${PERMS})
