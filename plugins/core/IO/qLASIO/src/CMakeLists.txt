
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/LasPlugin.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasIOFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasOpenDialog.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasSaveDialog.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasDetails.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldLoader.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldSaver.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasWaveformLoader.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasWaveformSaver.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasSavedInfo.cpp
        )
