#!/bin/bash

# Use install_name_tool to change paths for some libraries so they can be found

FRAMEWORK_DIR=$1

# This is specific to my (Andy's) build setup because I install all my homebrew stuff here.
# If you are trying to package things yourself for general deployment, you'll have to change this path.
HOMEBREW_PATH_PREFIX=${HOME}/dev

if [ -f "${FRAMEWORK_DIR}/libCGAL_Core.13.dylib" ]; then
   echo "  fixing: ${FRAMEWORK_DIR}/libCGAL_Core.13.dylib"
   install_name_tool -change "${HOMEBREW_PATH_PREFIX}/lib/libCGAL.13.dylib" "@executable_path/../Frameworks/libCGAL.13.dylib" "${FRAMEWORK_DIR}/libCGAL_Core.13.dylib"
else
   echo "  error: could not find ${FRAMEWORK_DIR}/libCGAL_Core.13.dylib - check versions in fix_libs.sh"
fi

echo "  fixing: ${FRAMEWORK_DIR}/libboost_chrono-mt.dylib"
install_name_tool -change "@loader_path/libboost_system-mt.dylib" "@executable_path/../Frameworks/libboost_system-mt.dylib" "${FRAMEWORK_DIR}/libboost_chrono-mt.dylib"

echo "  fixing: ${FRAMEWORK_DIR}/libboost_thread-mt.dylib"
install_name_tool -change "@loader_path/libboost_system-mt.dylib" "@executable_path/../Frameworks/libboost_system-mt.dylib" "${FRAMEWORK_DIR}/libboost_thread-mt.dylib"
