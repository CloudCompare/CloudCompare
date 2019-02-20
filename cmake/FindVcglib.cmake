# Ceres Solver - A fast non-linear least squares minimizer
# Copyright 2015 Google Inc. All rights reserved.
# http://ceres-solver.org/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Google Inc. nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: alexs.mac@gmail.com (Alex Stewart)
#

# FindVcglib.cmake - Find vcglib library
#
# This module defines the following variables:
#
# VCGLIB_FOUND: TRUE if vcglib is found.
# VCGLIB_INCLUDE_DIRS: Include directories for vcglib.
#
# The following variables control the behaviour of this module:
#
# VCGLIB_INCLUDE_DIR_HINTS: List of additional directories in which to
#                          search for vcglib includes, e.g: /timbuktu/vcglib.
#
# The following variables are also defined by this module, but in line with
# CMake recommended FindPackage() module style should NOT be referenced directly
# by callers (use the plural variables detailed above instead).  These variables
# do however affect the behaviour of the module via FIND_[PATH/LIBRARY]() which
# are NOT re-called (i.e. search for library is not repeated) if these variables
# are set with valid values _in the CMake cache_. This means that if these
# variables are set directly in the cache, either by the user in the CMake GUI,
# or by the user passing -DVAR=VALUE directives to CMake when called (which
# explicitly defines a cache variable), then they will be used verbatim,
# bypassing the HINTS variables and other hard-coded search locations.
#
# VCGLIB_INCLUDE_DIR: Include directory for CXSparse, not including the
#                    include directory of any dependencies.

# Called if we failed to find Eigen or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(vcglib_report_not_found REASON_MSG)
  unset(VCGLIB_FOUND)
  unset(VCGLIB_INCLUDE_DIRS)
  # Make results of search visible in the CMake GUI if Eigen has not
  # been found so that user does not have to toggle to advanced view.
  mark_as_advanced(CLEAR VCGLIB_INCLUDE_DIR)
  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
  # use the camelcase library name, not uppercase.
  if (VCGLIB_FIND_QUIETLY)
    message(STATUS "Failed to find Vcglib - " ${REASON_MSG} ${ARGN})
  elseif (VCGLIB_FIND_REQUIRED)
    message(FATAL_ERROR "Failed to find Vcglib - " ${REASON_MSG} ${ARGN})
  else()
    # Neither QUIETLY nor REQUIRED, use no priority which emits a message
    # but continues configuration and allows generation.
    message("-- Failed to find Vcglib - " ${REASON_MSG} ${ARGN})
  endif ()
endmacro(vcglib_report_not_found)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
#
# TODO: Add standard Windows search locations for vcglib.
list(APPEND VCGLIB_CHECK_INCLUDE_DIRS
  /usr/local/include
  /usr/local/homebrew/include # Mac OS X
  /opt/local/var/macports/software # Mac OS X.
  /opt/local/include
  /usr/include)
# Additional suffixes to try appending to each search path.
list(APPEND VCGLIB_CHECK_PATH_SUFFIXES
  vcglib) # Default root directory for vcglib.

# Search supplied hint directories first if supplied.
find_path(VCGLIB_INCLUDE_DIR
  NAMES wrap/utils.h
  HINTS ${VCGLIB_INCLUDE_DIR_HINTS}
  PATHS ${VCGLIB_CHECK_INCLUDE_DIRS}
  PATH_SUFFIXES ${VCGLIB_CHECK_PATH_SUFFIXES})

if (NOT VCGLIB_INCLUDE_DIR OR
    NOT EXISTS ${VCGLIB_INCLUDE_DIR})
  vcglib_report_not_found(
    "Could not find vcglib include directory, set VCGLIB_INCLUDE_DIR to "
    "path to vcglib include directory")
endif (NOT VCGLIB_INCLUDE_DIR OR
       NOT EXISTS ${VCGLIB_INCLUDE_DIR})

# Mark internally as found, then verify. vcglib_report_not_found() unsets
# if called.
set(VCGLIB_FOUND TRUE)

# Set standard CMake FindPackage variables if found.
if (VCGLIB_FOUND)
  set(VCGLIB_INCLUDE_DIRS ${VCGLIB_INCLUDE_DIR})
endif (VCGLIB_FOUND)

# Handle REQUIRED / QUIET optional arguments and version.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Vcglib
  REQUIRED_VARS VCGLIB_INCLUDE_DIRS)

# Only mark internal variables as advanced if we found Vcglib, otherwise
# leave it visible in the standard GUI for the user to set manually.
if (VCGLIB_FOUND)
  mark_as_advanced(FORCE VCGLIB_INCLUDE_DIR)
endif (VCGLIB_FOUND)
