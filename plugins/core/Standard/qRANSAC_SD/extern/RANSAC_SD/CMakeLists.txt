project( QRANSAC_SD_PRIM_SHAPES_LIB VERSION 1.1 )

add_library( ${PROJECT_NAME} STATIC )

add_subdirectory( GfxTL )
add_subdirectory( MiscLib )

target_compile_definitions( ${PROJECT_NAME}
	PUBLIC
		# Causes sub-clouds to be generated as partial clones of original cloud
		POINTSWITHINDEX
	PRIVATE
		PRECISIONLEVMAR
		_CRT_SECURE_NO_DEPRECATE
		_CRT_SECURE_NO_WARNINGS
		_SCL_SECURE_NO_WARNINGS
		_LIB
		$<$<CONFIG:Debug>:TIMINGLEVEL0>
		$<$<CONFIG:Release>:TIMINGLEVEL1>
)

# Apparently building with OpenMP is broken on Ubuntu
# DGM: OpenMP doesn't work with MSVC (the process loops infinitely)
#if( NOT WIN32 )
#	find_package( OpenMP )

#	if( OpenMP_CXX_FOUND )
#		target_link_libraries( ${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX )
#		target_compile_definitions( ${PROJECT_NAME} PRIVATE DOPARALLEL )
#	endif()
#endif()

target_sources( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/BasePrimitiveShape.h
		${CMAKE_CURRENT_LIST_DIR}/basic.h
		${CMAKE_CURRENT_LIST_DIR}/Bitmap.h
		${CMAKE_CURRENT_LIST_DIR}/BitmapPrimitiveShape.h
		${CMAKE_CURRENT_LIST_DIR}/Candidate.h
		${CMAKE_CURRENT_LIST_DIR}/Cone.h
		${CMAKE_CURRENT_LIST_DIR}/ConePrimitiveShape.h
		${CMAKE_CURRENT_LIST_DIR}/ConePrimitiveShapeConstructor.h
		${CMAKE_CURRENT_LIST_DIR}/Cylinder.h
		${CMAKE_CURRENT_LIST_DIR}/CylinderPrimitiveShape.h
		${CMAKE_CURRENT_LIST_DIR}/CylinderPrimitiveShapeConstructor.h
		${CMAKE_CURRENT_LIST_DIR}/FlatNormalThreshPointCompatibilityFunc.h
		${CMAKE_CURRENT_LIST_DIR}/Grid.h
		${CMAKE_CURRENT_LIST_DIR}/IndexIterator.h
		${CMAKE_CURRENT_LIST_DIR}/LevMarFitting.h
		${CMAKE_CURRENT_LIST_DIR}/LevMarFunc.h
		${CMAKE_CURRENT_LIST_DIR}/LevMarLSWeight.h
		${CMAKE_CURRENT_LIST_DIR}/LowStretchSphereParametrization.h
		${CMAKE_CURRENT_LIST_DIR}/LowStretchTorusParametrization.h
		${CMAKE_CURRENT_LIST_DIR}/Octree.h
		${CMAKE_CURRENT_LIST_DIR}/Plane.h
		${CMAKE_CURRENT_LIST_DIR}/PlanePrimitiveShape.h
		${CMAKE_CURRENT_LIST_DIR}/PlanePrimitiveShapeConstructor.h
		${CMAKE_CURRENT_LIST_DIR}/PointCloud.h
		${CMAKE_CURRENT_LIST_DIR}/PrimitiveShape.h
		${CMAKE_CURRENT_LIST_DIR}/PrimitiveShapeConstructor.h
		${CMAKE_CURRENT_LIST_DIR}/PrimitiveShapeVisitor.h
		${CMAKE_CURRENT_LIST_DIR}/RansacShapeDetector.h
		${CMAKE_CURRENT_LIST_DIR}/RebuildAACubeTreeStrategy.h
		${CMAKE_CURRENT_LIST_DIR}/ScoreAACubeTreeStrategy.h
		${CMAKE_CURRENT_LIST_DIR}/ScoreComputer.h
		${CMAKE_CURRENT_LIST_DIR}/ScorePrimitiveShapeVisitor.h
		${CMAKE_CURRENT_LIST_DIR}/SimpleTorusParametrization.h
		${CMAKE_CURRENT_LIST_DIR}/Sphere.h
		${CMAKE_CURRENT_LIST_DIR}/SpherePrimitiveShape.h
		${CMAKE_CURRENT_LIST_DIR}/SpherePrimitiveShapeConstructor.h
		${CMAKE_CURRENT_LIST_DIR}/Torus.h
		${CMAKE_CURRENT_LIST_DIR}/TorusPrimitiveShape.h
		${CMAKE_CURRENT_LIST_DIR}/TorusPrimitiveShapeConstructor.h
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/Bitmap.cpp
		${CMAKE_CURRENT_LIST_DIR}/BitmapPrimitiveShape.cpp
		${CMAKE_CURRENT_LIST_DIR}/Candidate.cpp
		${CMAKE_CURRENT_LIST_DIR}/Cone.cpp
		${CMAKE_CURRENT_LIST_DIR}/ConePrimitiveShape.cpp
		${CMAKE_CURRENT_LIST_DIR}/ConePrimitiveShapeConstructor.cpp
		${CMAKE_CURRENT_LIST_DIR}/Cylinder.cpp
		${CMAKE_CURRENT_LIST_DIR}/CylinderPrimitiveShape.cpp
		${CMAKE_CURRENT_LIST_DIR}/CylinderPrimitiveShapeConstructor.cpp
		${CMAKE_CURRENT_LIST_DIR}/LowStretchSphereParametrization.cpp
		${CMAKE_CURRENT_LIST_DIR}/LowStretchTorusParametrization.cpp
		${CMAKE_CURRENT_LIST_DIR}/Plane.cpp
		${CMAKE_CURRENT_LIST_DIR}/PlanePrimitiveShape.cpp
		${CMAKE_CURRENT_LIST_DIR}/PlanePrimitiveShapeConstructor.cpp
		${CMAKE_CURRENT_LIST_DIR}/PointCloud.cpp
		${CMAKE_CURRENT_LIST_DIR}/RansacShapeDetector.cpp
		${CMAKE_CURRENT_LIST_DIR}/SimpleTorusParametrization.cpp
		${CMAKE_CURRENT_LIST_DIR}/solve.cpp
		${CMAKE_CURRENT_LIST_DIR}/Sphere.cpp
		${CMAKE_CURRENT_LIST_DIR}/SpherePrimitiveShape.cpp
		${CMAKE_CURRENT_LIST_DIR}/SpherePrimitiveShapeConstructor.cpp
		${CMAKE_CURRENT_LIST_DIR}/Torus.cpp
		${CMAKE_CURRENT_LIST_DIR}/TorusPrimitiveShape.cpp
		${CMAKE_CURRENT_LIST_DIR}/TorusPrimitiveShapeConstructor.cpp
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
)
