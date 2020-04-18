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

message(STATUS "tests")

enable_testing()

# --- ------------
# --- target tests

set(PYTHONAPI_TEST_DIRECTORY "/tmp" CACHE STRING "Directory path for tests: to store samples and generated data")
configure_file(gendata.py.in gendata.py)
configure_file(execBuildTest.sh.in execBuildTest.sh)

# --- define a function to simplify adding tests

function(do_test arg)
    add_test(NAME CCPAPI_${arg}
    COMMAND execBuildTest.sh ${arg}.py
    )
endfunction(do_test)

do_test(test001)
do_test(test002)
do_test(test003)
