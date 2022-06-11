add_subdirectory(las)
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/qPDALIO.h
        ${CMAKE_CURRENT_LIST_DIR}/qPDALIO.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ProgressStage.h
        ${CMAKE_CURRENT_LIST_DIR}/ProgressStage.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Compatibility.h
        )
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
