# ------------------------------------------------------------------------------
# qHull+CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_QUICKHULL "Build with qHull (convex hull generation library)" OFF )
if( ${OPTION_USE_QUICKHULL} )

	add_subdirectory (contrib/qhull/src2012.1)

endif()
