cmake_minimum_required(VERSION 3.0)

project( CloudComparePlugins )

# reset buffers (for MacOS bundle)
set( CLOUDCOMPARE_PLUGINS CACHE INTERNAL "CloudCompare plugin list" )
set( CCVIEWER_PLUGINS CACHE INTERNAL "ccViewer plugin list" )

set( CloudCompare_SOURCE_DIR "${CMAKE_SOURCE_DIR}/qCC" )
set( CloudComparePlugins_SOURCE_DIR "${CMAKE_SOURCE_DIR}/plugins" )

add_subdirectory( core )
add_subdirectory( example )
add_subdirectory( 3rdParty )
