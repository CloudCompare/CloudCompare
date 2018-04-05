cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

project( QPOISSON_RECON_LIB )

#to remove very annoying "NO ARGUMENTS THAT DEPEND ON A TEMPLATE PARAMETER" errors with G++!
if( UNIX OR MINGW )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")
endif()

set( SAMPLE_PROJECT_VERSION_MAJOR 9 )
set( SAMPLE_PROJECT_VERSION_MINOR 1 )

file( GLOB header_list Src_CC_wrap/*.h )
list( APPEND header_list Src/Allocator.h Src/Array.h Src/BinaryNode.h Src/BSplineData.h Src/CmdLineParser.h Src/Factor.h Src/FunctionData.h Src/Geometry.h Src/MarchingCubes.h Src/MAT.h Src/MemoryUsage.h Src/MultiGridOctreeData.h Src/MyTime.h Src/Octree.h Src/Ply.h Src/PointStream.h Src/Polynomial.h Src/PPolynomial.h Src/SparseMatrix.h )
list( APPEND inline_list Src/Array.inl Src/BSplineData.inl Src/CmdLineParser.inl Src/FunctionData.inl Src/Geometry.inl Src/MAT.inl Src/MultiGridOctreeData.inl Src/MultiGridOctreeData.IsoSurface.inl Src/MultiGridOctreeData.SortedTreeNodes.inl Src/Octree.inl Src/PointStream.inl Src/Polynomial.inl Src/PPolynomial.inl Src/SparseMatrix.inl )
file( GLOB source_list Src_CC_wrap/*.cpp )
list( APPEND source_list Src/CmdLineParser.cpp Src/Factor.cpp Src/Geometry.cpp Src/MarchingCubes.cpp Src/PlyFile.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${inline_list} ${source_list} )

# Add preprocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_DEPRECATE _CRT_SECURE_NO_WARNINGS NOMINMAX )

#option( POISSON_RECON_WITH_OPEN_MP "Check to compile PoissonRecon plugin with OpenMP support" OFF )
if ( POISSON_RECON_WITH_OPEN_MP )
	find_package(OpenMP)
	if (OPENMP_FOUND)
		set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
		set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS WITH_OPENMP )
	endif()
endif ()
