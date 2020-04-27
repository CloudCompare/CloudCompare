project( CloudComparePlugins )

# Add our cmake module path so we don't need relative paths for these
list( APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" )

include( Plugins )

# reset buffers (for MacOS bundle)
set( CLOUDCOMPARE_PLUGINS CACHE INTERNAL "CloudCompare plugin list" )
set( CCVIEWER_PLUGINS CACHE INTERNAL "ccViewer plugin list" )

set( CloudCompare_SOURCE_DIR "${CMAKE_SOURCE_DIR}/qCC" )
set( CloudComparePlugins_SOURCE_DIR "${CMAKE_SOURCE_DIR}/plugins" )

add_subdirectory( core )
add_subdirectory( example )
add_subdirectory( 3rdParty )
