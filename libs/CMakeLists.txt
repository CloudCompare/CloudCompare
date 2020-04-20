add_subdirectory( CCFbo )
add_subdirectory( qCC_db ) # must always be included after CCFbo (dependency)
add_subdirectory( qCC_io ) # must always be included after qCC_db (dependency)
add_subdirectory( qCC_glWindow ) # must always be included after qCC_db (dependency)
