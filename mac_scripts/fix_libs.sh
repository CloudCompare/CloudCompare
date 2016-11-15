#!/bin/bash

# Use install_name_tool to change paths for some libraries so they can be found

FRAMEWORK_DIR=$1

# This is specific to my (Andy's) build setup because I install all my homebrew stuff here.
# If you are trying to package things yourself for general deployment, you'll have to change this path.
PATH_PREFIX=${HOME}/dev

echo "  fixing: ${FRAMEWORK_DIR}/libCGAL_Core.12.dylib"
install_name_tool -change "${PATH_PREFIX}/lib/libCGAL.12.dylib" "@executable_path/../Frameworks/libCGAL.12.dylib" "${FRAMEWORK_DIR}/libCGAL_Core.12.dylib"

echo "  fixing: ${FRAMEWORK_DIR}/libboost_chrono-mt.dylib"
install_name_tool -change "@loader_path/libboost_system-mt.dylib" "@executable_path/../Frameworks/libboost_system-mt.dylib" "${FRAMEWORK_DIR}/libboost_chrono-mt.dylib"

echo "  fixing: ${FRAMEWORK_DIR}/libboost_thread-mt.dylib"
install_name_tool -change "@loader_path/libboost_system-mt.dylib" "@executable_path/../Frameworks/libboost_system-mt.dylib" "${FRAMEWORK_DIR}/libboost_thread-mt.dylib"
