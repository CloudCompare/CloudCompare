project( qhull )

add_library( ${PROJECT_NAME} STATIC )

target_sources( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/geom.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/io.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/libqhull.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/mem.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/merge.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/poly.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/random.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/qhull_a.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/qset.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/stat.h
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/user.h
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/rboxlib.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/user.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/global.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/stat.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/io.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/geom2.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/poly2.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/merge.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/libqhull.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/geom.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/poly.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/qset.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/mem.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/usermem.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/userprintf.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/userprintf_rbox.c
		${CMAKE_CURRENT_SOURCE_DIR}/qhull/random.c
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/qhull
)
