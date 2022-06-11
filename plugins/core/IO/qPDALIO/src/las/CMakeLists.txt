target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/LasDetails.h
        ${CMAKE_CURRENT_LIST_DIR}/LasDetails.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/LasFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasFields.h
        ${CMAKE_CURRENT_LIST_DIR}/LasFields.cpp
        ${CMAKE_CURRENT_LIST_DIR}/PdalToCC.h
        ${CMAKE_CURRENT_LIST_DIR}/PdalToCC.cpp
        ${CMAKE_CURRENT_LIST_DIR}/CCToPdal.h
        ${CMAKE_CURRENT_LIST_DIR}/CCToPdal.cpp
        )

add_subdirectory(ui)


target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})