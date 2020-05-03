project( RandomKitLib VERSION 1.6 )

set( CMAKE_AUTOMOC OFF )

add_library( ${PROJECT_NAME} STATIC )

target_sources( ${PROJECT_NAME}
    PRIVATE
        Randomkit/erfc.h
        Randomkit/rk_isaac.c
        Randomkit/rk_mt.c
        Randomkit/rk_primitive.c
        Randomkit/rk_sobol.c
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    Randomkit
)
