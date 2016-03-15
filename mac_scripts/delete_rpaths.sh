#!/bin/bash

# Use install_name_tool to remove developer-specific paths from the libraries and executable.

# Note from Andy [15 March 2016]:
#        I don't know how to prevent these RPATHS from being added to the libs through cmake.
#        So I must remove them "by hand".
#        The macdeploqt/cmake/BundleUtilities/rpath confusion has defeated me.

EXECUTABLE=$1
FRAMEWORK_DIR=$2
PLUGIN_DIR=$3

# This is specific to my (Andy's) build setup because I install all my homebrew stuff here.
# If you are trying to package things yourself, you'll have to change this.
PATH_PREFIX=${HOME}/dev

echo "* Remove extra RPATHS in executable: ${EXECUTABLE}"
install_name_tool -delete_rpath "${PATH_PREFIX}/lib" "${EXECUTABLE}"

echo "* Remove extra RPATHS in dylibs from ${FRAMEWORK_DIR}"

echo "  fixing: ${FRAMEWORK_DIR}/libCC_CORE_LIB.dylib"
install_name_tool -delete_rpath "${PATH_PREFIX}/Cellar/cgal/4.7/lib" -delete_rpath "${PATH_PREFIX}/lib" "${FRAMEWORK_DIR}/libCC_CORE_LIB.dylib"

echo "  fixing: ${FRAMEWORK_DIR}/libQCC_DB_LIB.dylib"
install_name_tool -delete_rpath "${PATH_PREFIX}/lib" "${FRAMEWORK_DIR}/libQCC_DB_LIB.dylib"

echo "  fixing: ${FRAMEWORK_DIR}/libQCC_IO_LIB.dylib"
install_name_tool -delete_rpath "${PATH_PREFIX}/lib" "${FRAMEWORK_DIR}/libQCC_IO_LIB.dylib"

echo "* Remove extra RPATHS in dylibs from ${PLUGIN_DIR}"

for dir in ${PLUGIN_DIR} ; do
   for file in "${dir}/"*.dylib ; do
     echo "  fixing: $file"
     install_name_tool -delete_rpath "${PATH_PREFIX}/lib" "$file"
   done
done
