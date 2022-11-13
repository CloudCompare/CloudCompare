option(PLUGIN_IO_QLAS "LAS IO plugin using laszip" OFF)

if (PLUGIN_IO_QLAS)
    project(QLAS_IO_PLUGIN)

    AddPlugin(NAME ${PROJECT_NAME} TYPE io)

    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    find_package(LASzip MODULE REQUIRED)

    target_compile_definitions(${PROJECT_NAME} PRIVATE -DQT_FORCE_ASSERTS)

    target_link_libraries(${PROJECT_NAME} LASzip::LASzip)

    add_subdirectory(include)
    add_subdirectory(src)
    add_subdirectory(ui)

    if (WIN32)
        copy_files( "${LASZIP_DLL}" "${CLOUDCOMPARE_DEST_FOLDER}" 1 )
    endif ()
endif ()
