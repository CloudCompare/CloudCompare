
target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/BundlerCommand.h
        ${CMAKE_CURRENT_LIST_DIR}/BundlerImportDlg.h
        ${CMAKE_CURRENT_LIST_DIR}/BundlerFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/IcmFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/PNFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/PovFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/PVFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/qAdditionalIO.h
        ${CMAKE_CURRENT_LIST_DIR}/SalomeHydroFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/SinusxFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/SoiFilter.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
