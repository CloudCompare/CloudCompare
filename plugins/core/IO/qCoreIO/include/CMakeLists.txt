
target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/HeightProfileFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/MAFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/MascaretFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/ObjFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/OFFFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/PTXFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/qCoreIO.h
        ${CMAKE_CURRENT_LIST_DIR}/SimpleBinFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/STLFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/VTKFilter.h

        ${CMAKE_CURRENT_LIST_DIR}/PDMS/PDMSFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/PDMS/PdmsParser.h
        ${CMAKE_CURRENT_LIST_DIR}/PDMS/PdmsTools.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/PDMS
)
