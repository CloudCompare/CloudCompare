
target_sources( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/AlignedAllocator.h
		${CMAKE_CURRENT_LIST_DIR}/NoShrinkVector.h
		${CMAKE_CURRENT_LIST_DIR}/Pair.h
		${CMAKE_CURRENT_LIST_DIR}/Performance.h
		${CMAKE_CURRENT_LIST_DIR}/Random.h
		${CMAKE_CURRENT_LIST_DIR}/RefCount.h
		${CMAKE_CURRENT_LIST_DIR}/RefCounted.h
		${CMAKE_CURRENT_LIST_DIR}/RefCountPtr.h
		${CMAKE_CURRENT_LIST_DIR}/Vector.h
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/Random.cpp
		${CMAKE_CURRENT_LIST_DIR}/RefCount.cpp
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
