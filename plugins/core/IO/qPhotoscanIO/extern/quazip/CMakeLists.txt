# Create a new CMake file for CloudCompare since the included ones are old-style

project( quazip )

find_package( ZLIB REQUIRED )
find_package( Qt5 COMPONENTS Core REQUIRED )

add_library( ${PROJECT_NAME} STATIC )

target_sources( ${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/ioapi.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quaadler32.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quachecksum32.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quacrc32.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quagzipfile.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quaziodevice.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazip_global.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazip.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazipdir.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazipfile.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazipfileinfo.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazipnewinfo.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/unzip.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/zip.h
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/crypt.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/JlCompress.h
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/JlCompress.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/qioapi.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quaadler32.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quacrc32.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quagzipfile.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quaziodevice.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazip.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazipdir.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazipfile.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazipfileinfo.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/quazipnewinfo.cpp
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/unzip.c
        ${CMAKE_CURRENT_LIST_DIR}/quazip/quazip/zip.c
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_SOURCE_DIR}/quazip/quazip
)

target_compile_definitions( ${PROJECT_NAME} PUBLIC QUAZIP_STATIC )

target_link_libraries( ${PROJECT_NAME} Qt5::Core ZLIB::ZLIB )
