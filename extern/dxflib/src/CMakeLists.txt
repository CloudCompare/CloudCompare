target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/dl_writer.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_writer_ascii.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_writer_ascii.cpp
        ${CMAKE_CURRENT_LIST_DIR}/dl_global.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_extrusion.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_entities.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_exception.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_dxf.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_dxf.cpp
        ${CMAKE_CURRENT_LIST_DIR}/dl_creationinterface.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_creationadapter.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_codes.h
        ${CMAKE_CURRENT_LIST_DIR}/dl_attributes.h
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
