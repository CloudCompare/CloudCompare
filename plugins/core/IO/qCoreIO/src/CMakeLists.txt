
target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/HeightProfileFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/MAFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/MascaretFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ObjFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/OFFFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/OFFFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/PTXFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/qCoreIO.cpp
        ${CMAKE_CURRENT_LIST_DIR}/SimpleBinFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/STLFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/VTKFilter.cpp
        
        ${CMAKE_CURRENT_LIST_DIR}/PDMS/PDMSFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/PDMS/PdmsParser.cpp
        ${CMAKE_CURRENT_LIST_DIR}/PDMS/PdmsTools.cpp
)
