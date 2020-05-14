cmake_minimum_required(VERSION 3.0)

option( PLUGIN_IO_QRDB "Check to install RDB 2 I/O plugin" OFF )

# CloudCompare RDB 2 I/O plugin
if (PLUGIN_IO_QRDB)

	project( QRDB_IO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	include( CMakePolicies NO_POLICY_SCOPE )

	# find rdb
	# if not defined globally define path to rdb-config.cmake by settin the
	# cmake variable rdb_DIR
	find_package(rdb REQUIRED)

	file( GLOB PLUGIN_HDR_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.h )
	file( GLOB PLUGIN_SRC_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp )
	file( GLOB PLUGIN_UI_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.ui )
	
	# add source files
	target_sources( ${PROJECT_NAME}
		PRIVATE
			${PLUGIN_HDR_LIST}
			${PLUGIN_SRC_LIST}
			${PLUGIN_UI_LIST}
	)

    include( CMakePluginTemplate )
	#include( ../../CMakePluginTpl.cmake )
	
	target_link_libraries(${PROJECT_NAME} rdbcpp)
endif()
