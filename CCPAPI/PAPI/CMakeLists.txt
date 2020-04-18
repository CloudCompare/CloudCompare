##########################################################################
#                                                                        #
#                               PAPI                                     #
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

message(STATUS "Python API")

# --- -----------------------------------------
# --- target C++ library cc Python Api (ccPApi)

project( CC_PAPI_LIB VERSION 0.1 )

set( CC_PAPI_HEADER_LIST
    ccPApi.h
    ccTrace.h
    )
    
set( CC_PAPI_SOURCE_LIST
    ccPApi.cpp
    )
    
add_library( CC_PAPI_LIB STATIC ${CC_PAPI_HEADER_LIST} ${CC_PAPI_SOURCE_LIST} )
    
target_include_directories( CC_PAPI_LIB PUBLIC
    ${CC_FBO_LIB_SOURCE_DIR}/include
    ${CC_CORE_LIB_SOURCE_DIR}/include
    ${QCC_DB_LIB_SOURCE_DIR}
    )
    
if( MSVC )
   # TODO is this order of include relevant ? I use the same order as in qCC (CloudCompare target) 
   target_include_directories( CC_PAPI_LIB PUBLIC
        ${QCC_DB_LIB_SOURCE_DIR}/msvc
        )
endif()

target_include_directories( CC_PAPI_LIB PUBLIC
    ${QCC_IO_LIB_SOURCE_DIR}
    ${QCC_GL_LIB_SOURCE_DIR}
    ${CloudComparePlugins_SOURCE_DIR} 
    ${CloudCompare_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common
    ${CMAKE_CURRENT_SOURCE_DIR}/db_tree
    ${CMAKE_CURRENT_SOURCE_DIR}/ui_templates
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/qcustomplot
    ${CMAKE_CURRENT_BINARY_DIR}
    )

target_link_libraries( CC_PAPI_LIB
    CC_FBO_LIB
    CC_CORE_LIB
    QCC_DB_LIB
    QCC_IO_LIB
    QCC_GL_LIB
    qcustomplot
    )

