##########################################################################
#                                                                        #
#                               CCPAPI TEST                              #
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

cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0071 NEW)
message(STATUS "tests")

# --- ------------
# --- target tests

set(CC_PYTHONAPI_TEST_DIRECTORY "/tmp" CACHE STRING "Directory path for tests: to store samples and generated data")
configure_file(gendata.py.in gendata.py)




