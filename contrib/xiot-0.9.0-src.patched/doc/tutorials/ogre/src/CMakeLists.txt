cmake_minimum_required(VERSION 2.6)

project(XOgre3D)
 
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

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Find the Ogre3D libraries
find_package(OGRE REQUIRED)
find_package(OIS)

# Find the X3DLoader libraries
set(XIOT_INCLUDE_DIR ${X3DLOADER_INCLUDE_DIR} CACHE PATH "")
set(XIOT_LIBRARY ${X3DLOADER_LIBRARY} CACHE FILEPATH  "")

find_library(XIOT_LIBRARY NAMES xiot DOC "The location of the XIOT library")

set(STEP1_SRC
	step1/main.cpp
	step1/TutorialApplication.cpp
	)

set(STEP2_SRC
	step2/main.cpp
	step2/TutorialApplication.cpp
	step2/TutorialApplication.h
	step2/OgreNodeHandler.cpp
	step2/OgreNodeHandler.h
	ArgumentHelper.cpp
	)

set(STEP3_SRC
	step3/main.cpp
	ArgumentHelper.cpp
	step3/TutorialApplication.cpp
	step3/OgreNodeHandler.h
	step3/OgreNodeHandler.cpp
	step3/GeomUtils.cpp
	step3/GeomUtils.h
	)

set(STEP4_SRC
	step4/main.cpp
	ArgumentHelper.cpp
	step4/TutorialApplication.cpp
	step4/OgreNodeHandler.h
	step4/OgreNodeHandler.cpp
	step4/GeomUtils.cpp
	step4/GeomUtils.h
	step4/IndexedGeometry.h
	step4/IndexedGeometry.cpp
	)

link_directories(${OGRE_LIB_DIR})
include_directories(${OGRE_INCLUDE_DIR} ${XIOT_INCLUDE_DIR} .)
	
add_executable(step1 ${STEP1_SRC})
target_link_libraries(step1 ${OGRE_LIBRARIES} ${OIS_LIBRARIES})

add_executable(step2 ${STEP2_SRC})
target_link_libraries(step2 ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${XIOT_LIBRARY})

add_executable(step3 ${STEP3_SRC})
target_link_libraries(step3 ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${XIOT_LIBRARY})

add_executable(step4 ${STEP4_SRC})
target_link_libraries(step4 ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${XIOT_LIBRARY})

CONFIGURE_FILE(cfg/resources.cfg.in
    ${EXECUTABLE_OUTPUT_PATH}/resources.cfg @ONLY)
CONFIGURE_FILE(cfg/Plugins.cfg.in
    ${EXECUTABLE_OUTPUT_PATH}/Plugins.cfg @ONLY)
CONFIGURE_FILE(cfg/ogre.cfg.in
    ${EXECUTABLE_OUTPUT_PATH}/ogre.cfg @ONLY)
    
