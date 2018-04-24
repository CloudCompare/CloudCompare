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

## FFMPEG from homebrew is problematic since it puts the "Cellar" paths directly in the libs
## Again, these paths need to be modified if you are shipping the version of CC you are installing...

if test -n "$(find ${FRAMEWORK_DIR} -maxdepth 1 -name 'libavcodec.*.dylib*' -print -quit)"; then
  FFMPEG_VERSION=`ffmpeg -version | head -n1 | sed "s/^.*version \([0-9.]*\) Copyright.*/\1/"`
  FFMPEG_DIR="${HOMEBREW_PATH_PREFIX}/Cellar/ffmpeg/${FFMPEG_VERSION}/lib"

  if [ ! -d "$FFMPEG_DIR" ]; then
    echo "  error: could not find FFMPEG directory: ${FFMPEG_DIR} - check versions in fix_libs.sh"
  else
    AV_CODEC="libavcodec.58.dylib"
    AV_FORMAT="libavformat.58.dylib"
    AV_UTIL="libavutil.56.dylib"

    SW_RESAMPLE="libswresample.3.dylib"
    SW_SCALE="libswscale.5.dylib"

    echo "  fixing: ${FRAMEWORK_DIR}/${AV_CODEC}"
    install_name_tool -change "${FFMPEG_DIR}/${AV_UTIL}" "@executable_path/../Frameworks/${AV_UTIL}" "${FRAMEWORK_DIR}/${AV_CODEC}"
    install_name_tool -change "${FFMPEG_DIR}/${SW_RESAMPLE}" "@executable_path/../Frameworks/${SW_RESAMPLE}" "${FRAMEWORK_DIR}/${AV_CODEC}"

    echo "  fixing: ${FRAMEWORK_DIR}/${AV_FORMAT}"
    install_name_tool -change "${FFMPEG_DIR}/${AV_CODEC}" "@executable_path/../Frameworks/${AV_CODEC}" "${FRAMEWORK_DIR}/${AV_FORMAT}"
    install_name_tool -change "${FFMPEG_DIR}/${AV_UTIL}" "@executable_path/../Frameworks/${AV_UTIL}" "${FRAMEWORK_DIR}/${AV_FORMAT}"
    install_name_tool -change "${FFMPEG_DIR}/${SW_RESAMPLE}" "@executable_path/../Frameworks/${SW_RESAMPLE}" "${FRAMEWORK_DIR}/${AV_FORMAT}"

    echo "  fixing: ${FRAMEWORK_DIR}/${SW_RESAMPLE}"
    install_name_tool -change "${FFMPEG_DIR}/${AV_UTIL}" "@executable_path/../Frameworks/${AV_UTIL}" "${FRAMEWORK_DIR}/${SW_RESAMPLE}"

    echo "  fixing: ${FRAMEWORK_DIR}/${SW_SCALE}"
    install_name_tool -change "${FFMPEG_DIR}/${AV_UTIL}" "@executable_path/../Frameworks/${AV_UTIL}" "${FRAMEWORK_DIR}/${SW_SCALE}"
  fi
fi
