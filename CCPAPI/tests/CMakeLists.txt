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

# where to install the tests scripts
set(PYTHONAPI_TEST_SCRIPTS_DIRECTORY doc/PythonAPI_test)

# data sample generator and other utilities for test
configure_file(gendata.py.in gendata.py)

# shell scripts to execute a test, a build or install step
if(WIN32)
    configure_file(execInstallTest.bat.in execInstallTest.bat)
    set(EXEC_TEST execTest.bat)
	set(EXEC_INSTALL_TEST execInstallTest.bat)
else()
    configure_file(execTest.sh.in execTest.sh)
    configure_file(execInstallTest.sh.in execInstallTest.sh)
	set(EXEC_TEST execTest.sh)
    set(EXEC_INSTALL_TEST execInstallTest.sh)
endif()

# list of tests
set(PYTHONAPI_TEST_SCRIPTS
    test001.py
    test002.py
    test003.py
    )

# list of utilities
set(PYTHONAPI_TEST_UTILITIES
    ${CMAKE_CURRENT_BINARY_DIR}/gendata.py
    )

# files to install    
set(PYTHONAPI_TEST_FILES
    ${PYTHONAPI_TEST_UTILITIES}
    ${PYTHONAPI_TEST_SCRIPTS}
    )

install(PROGRAMS ${PYTHONAPI_TEST_FILES}
        DESTINATION ${PYTHONAPI_TEST_SCRIPTS_DIRECTORY}
       )

# the shell script executing a test is renamed a install step      
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${EXEC_INSTALL_TEST}
        DESTINATION ${PYTHONAPI_TEST_SCRIPTS_DIRECTORY}
        RENAME ${EXEC_TEST}
       )
        
# --- define a function to simplify adding tests

function(do_test arg)
    add_test(NAME CCPAPI_${arg}
    COMMAND ${EXEC_TEST} ${arg}.py
    )
endfunction(do_test)

do_test(test001)
do_test(test002)
do_test(test003)

    