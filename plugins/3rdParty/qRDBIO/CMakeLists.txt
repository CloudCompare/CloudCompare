cmake_minimum_required(VERSION 3.0)

option( PLUGIN_IO_QRDB "Check to install RDB 2 I/O plugin" OFF )

# CloudCompare RDB 2 I/O plugin
if (PLUGIN_IO_QRDB)
	project( QRDB_IO_PLUGIN )

	include( CMakePolicies NO_POLICY_SCOPE )

	# find rdb
	# if not defined globally define path to rdb-config.cmake by settin the
	# cmake variable rdb_DIR
	find_package(rdb REQUIRED)

	# Set to include this I/O plugin in ccViewer
	set( CC_IS_IO_PLUGIN 1 )

    #include( CMakePluginTemplate )
	include( ../../CMakePluginTpl.cmake )
	
	target_link_libraries(${PROJECT_NAME} rdbcpp)
endif()
