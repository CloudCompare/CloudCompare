cmake_minimum_required( VERSION 3.1 )

option( PLUGIN_IO_QGLTF "Install qGLTF_IO plugin to read & write GLTF files using fx-gltf" OFF )

if ( PLUGIN_IO_QGLTF )
	project( QGLTF_IO_PLUGIN )

	include ( ${CC_CMAKE_SCRIPTS_DIR}/CMakePolicies.cmake NO_POLICY_SCOPE )

	set( CC_IS_IO_PLUGIN 1 )

	find_package( nlohmann_json 3.2.0 REQUIRED )

	include_directories( extern/fx-gltf/include )
	include_directories( src )

	add_subdirectory( src )
	add_subdirectory( extern/fx-gltf EXCLUDE_FROM_ALL)

	include( ../../../CMakePluginTpl.cmake )

	target_link_libraries( ${PROJECT_NAME} nlohmann_json::nlohmann_json fx-gltf::fx-gltf )

endif()
