cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

project( PoissonRecon )

#to remove very annoying "NO ARGUMENTS THAT DEPEND ON A TEMPLATE PARAMETER" errors with G++!
if( UNIX OR MINGW )
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")
endif()

set( SAMPLE_PROJECT_VERSION_MAJOR 5 )
set( SAMPLE_PROJECT_VERSION_MINOR 5 )

file( GLOB header_list Src_CC_wrap/*.h )
list (APPEND header_list Src/Allocator.h Src/Array.h Src/BinaryNode.h Src/BSplineData.h Src/CmdLineParser.h Src/Factor.h Src/Geometry.h Src/Hash.h Src/MarchingCubes.h Src/MAT.h Src/MemoryUsage.h Src/MultiGridOctreeData.h Src/Octree.h Src/Ply.h Src/PlyFile.h Src/PointStream.h Src/Polynomial.h Src/PPolynomial.h Src/SparseMatrix.h Src/poissonTime.h Src/Vector.h )
list (APPEND header_list Src/Array.inl Src/CmdLineParser.inl Src/MAT.inl Src/PointStream.inl Src/BSplineData.inl Src/Geometry.inl Src/MultiGridOctreeData.inl Src/Octree.inl Src/Polynomial.inl Src/PPolynomial.inl Src/SparseMatrix.inl Src/Vector.inl )
file( GLOB source_list Src_CC_wrap/*.cpp )
list (APPEND source_list Src/CmdLineParser.cpp Src/Factor.cpp Src/Geometry.cpp Src/MarchingCubes.cpp Src/PlyFile.cpp Src/poissonTime.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${inline_list} ${source_list} )

# Add prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_DEPRECATE _CRT_SECURE_NO_WARNINGS NOMINMAX )

option( POISSON_RECON_WITH_OPEN_MP "Check to compile PoissonRecon plugin with OpenMP support" OFF )
if ( POISSON_RECON_WITH_OPEN_MP )
	find_package(OpenMP)
	if (OPENMP_FOUND)
		set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
		set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS WITH_OPENMP )
	endif()
endif ()
