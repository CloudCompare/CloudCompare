add_subdirectory( qAnimation )
add_subdirectory( qBroom )
add_subdirectory( qCanupo )
add_subdirectory( qCloudLayers )
add_subdirectory( qCompass )
add_subdirectory( qCork )
add_subdirectory( qCSF )
add_subdirectory( qFacets )
add_subdirectory( qHoughNormals )
add_subdirectory( qHPR )
add_subdirectory( qM3C2 )
add_subdirectory( qPCL )
add_subdirectory( qPCV )
add_subdirectory( qPoissonRecon )
add_subdirectory( qRANSAC_SD )
add_subdirectory( qSRA )
add_subdirectory( qMeshBoolean )

#plugins integrated as submodules
set( submod_plugins
		${CMAKE_CURRENT_SOURCE_DIR}/qColorimetricSegmenter
		${CMAKE_CURRENT_SOURCE_DIR}/qMasonry
		${CMAKE_CURRENT_SOURCE_DIR}/qMPlane
		${CMAKE_CURRENT_SOURCE_DIR}/qJSonRPCPlugin
		${CMAKE_CURRENT_SOURCE_DIR}/qTreeIso
		${CMAKE_CURRENT_SOURCE_DIR}/q3DMASC
)

foreach( dir ${submod_plugins} )
    if( IS_DIRECTORY ${dir} AND EXISTS ${dir}/CMakeLists.txt )
		message( STATUS "Found submodule plugin: " ${dir} )
		add_subdirectory( ${dir} )
	endif()
endforeach()
