##########################################################################
#                                                                        #
#                               CMAKE / CLOUDCOMPARE                     #
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


# - Find PYQT and SIP
# Sets the following variables:
#   PYQT_PYUIC_EXECUTABLE  - path to pyuic5 script           : pyuic5
#   PYQT_PYRCC_EXECUTABLE  - path to pyrcc5 script           : pyrcc5
#   PYQT_PYTHONPATH        - path to PyQt5 Python module
#   SIP_EXECUTABLE         - path to the SIP executable      : sip
#   SIP_INCLUDE_DIR        - path to the SIP headers         : sip.h
#   SIP_PYTHONPATH         - path to the SIP Python packages : sipconfig.py (not required)
#   PYQT_SIPFLAGS
#

IF(NOT PYQTSIP_FIND_QUIETLY)
  MESSAGE(STATUS "Looking for PYQT5 and SIP ...")
ENDIF()

FIND_PROGRAM(PYQT_PYUIC_EXECUTABLE NAMES pyuic5 pyuic5.bat DOC "full path of pyuic5 script")
FIND_PROGRAM(PYQT_PYRCC_EXECUTABLE NAMES pyrcc5 pyrcc5.bat DOC "full path of pyrcc5 script")

EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} -c "import sys;
hints=' '.join(sys.path);
sys.stdout.write(hints)"
OUTPUT_VARIABLE _tmpPythonPath)
SEPARATE_ARGUMENTS(_tmpPythonPath)
FIND_PATH(PYQT_PYTHONPATH PyQt5/__init__.py HINTS ${_tmpPythonPath} DOC "directory path of PyQt5 Python module")
MESSAGE(STATUS "PYQT_PYTHONPATH: " ${PYQT_PYTHONPATH})

FIND_PROGRAM(SIP_EXECUTABLE sip DOC "full path of executable: sip")
MESSAGE("SIP_EXECUTABLE: " ${SIP_EXECUTABLE})

FIND_PATH(SIP_INCLUDE_DIR sip.h PATH_SUFFIXES python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} DOC "directory path of include: sip.h")
MESSAGE(STATUS "SIP_INCLUDE_DIR: " ${SIP_INCLUDE_DIR})

FIND_PATH(PYQT_SIPS QtGui/QtGuimod.sip HINTS "/usr/share/sip/PyQt5" DOC "directory path of QtGui/QtGuimod.sip")
MESSAGE(STATUS "PYQT_SIPS: " ${PYQT_SIPS})

IF(SIP_INCLUDE_DIR)
  GET_FILENAME_COMPONENT(SIP_PYTHONPATH "${SIP_INCLUDE_DIR}" PATH)
  GET_FILENAME_COMPONENT(SIP_PYTHONPATH "${SIP_PYTHONPATH}" PATH)
  SET(SIP_PYTHONPATH "${SIP_PYTHONPATH}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages")
  MESSAGE(STATUS "SIP_PYTHONPATH: " ${SIP_PYTHONPATH})
ENDIF()

EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} -c "import sys; 
sys.path[:0] = '${PYQT_PYTHONPATH}'.split(';');
from PyQt5.QtCore import PYQT_CONFIGURATION;
sys.stdout.write(PYQT_CONFIGURATION['sip_flags'])"
OUTPUT_VARIABLE PYQT_SIPFLAGS)
SEPARATE_ARGUMENTS(PYQT_SIPFLAGS)

SET(PYQT_SIPFLAGS ${PYQT_SIPFLAGS} -I "${PYQT_SIPS}")
MESSAGE(STATUS "PYQT_SIPFLAGS: " "${PYQT_SIPFLAGS}")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PYQTSIP REQUIRED_VARS PYQT_PYUIC_EXECUTABLE SIP_INCLUDE_DIR SIP_EXECUTABLE PYQT_SIPFLAGS)


