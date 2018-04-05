cmake_minimum_required(VERSION 3.0)

project( CloudComparePlugins )

# reset buffers (for MacOS bundle)
set( CLOUDCOMPARE_PLUGINS CACHE INTERNAL "CloudCompare plugin list" )
set( CCVIEWER_PLUGINS CACHE INTERNAL "ccViewer plugin list" )

add_subdirectory( core )
