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

## FFMPEG from homebrew is problematic since it puts the "Cellar" paths directly in the libs
## Again, these paths need to be modified if you are shipping the version of CC you are installing...

if [ -f "${FRAMEWORK_DIR}/libavcodec.57.dylib" ]; then
  FFMPEG_VERSION=`ffmpeg -version | head -n1 | sed "s/^.*version \([0-9.]*\) Copyright.*/\1/"`
  FFMPEG_DIR="${PATH_PREFIX}/Cellar/ffmpeg/${FFMPEG_VERSION}/lib"

  echo "  fixing: ${FRAMEWORK_DIR}/libavcodec.57.dylib"
  install_name_tool -change "${FFMPEG_DIR}/libavutil.55.dylib" "@executable_path/../Frameworks/libavutil.55.dylib" "${FRAMEWORK_DIR}/libavcodec.57.dylib"
  install_name_tool -change "${FFMPEG_DIR}/libswresample.2.dylib" "@executable_path/../Frameworks/libswresample.2.dylib" "${FRAMEWORK_DIR}/libavcodec.57.dylib"

  echo "  fixing: ${FRAMEWORK_DIR}/libavformat.57.dylib"
  install_name_tool -change "${FFMPEG_DIR}/libavcodec.57.dylib" "@executable_path/../Frameworks/libavcodec.57.dylib" "${FRAMEWORK_DIR}/libavformat.57.dylib"
  install_name_tool -change "${FFMPEG_DIR}/libavutil.55.dylib" "@executable_path/../Frameworks/libavutil.55.dylib" "${FRAMEWORK_DIR}/libavformat.57.dylib"
  install_name_tool -change "${FFMPEG_DIR}/libswresample.2.dylib" "@executable_path/../Frameworks/libswresample.2.dylib" "${FRAMEWORK_DIR}/libavformat.57.dylib"
  
  echo "  fixing: ${FRAMEWORK_DIR}/libswresample.2.dylib"
  install_name_tool -change "${FFMPEG_DIR}/libavutil.55.dylib" "@executable_path/../Frameworks/libavutil.55.dylib" "${FRAMEWORK_DIR}/libswresample.2.dylib"
  
  echo "  fixing: ${FRAMEWORK_DIR}/libswscale.4.dylib"
  install_name_tool -change "${FFMPEG_DIR}/libavutil.55.dylib" "@executable_path/../Frameworks/libavutil.55.dylib" "${FRAMEWORK_DIR}/libswscale.4.dylib"
fi
