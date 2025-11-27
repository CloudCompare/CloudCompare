set_directory_properties( PROPERTIES CLEAN_NO_CUSTOM 1 )

find_package(Qt6 COMPONENTS LinguistTools REQUIRED )

# Now handle our own .ts files
file( GLOB TRANSLATION_FILES ${CMAKE_CURRENT_LIST_DIR}/*.ts )


# Collect plugin files for available for translations.
# They are not part of the main CloudCompare target and may be built optionally.
file(GLOB_RECURSE PLUGIN_TRANSLATION_SOURCES
		${CMAKE_SOURCE_DIR}/plugins/core/*.cpp
		${CMAKE_SOURCE_DIR}/plugins/core/*.h
		${CMAKE_SOURCE_DIR}/plugins/core/*.ui
)

# Qt 6.2 qt_add_translations command.
# The release_translations target is built by default.
# The update_translations target can be invoked manually via CMake, e.g.:
#   cmake --build <build_dir> --target update_translations
qt_add_translations( ${PROJECT_NAME}
	TS_FILES ${TRANSLATION_FILES}
	SOURCES ${PLUGIN_TRANSLATION_SOURCES}
	LUPDATE_OPTIONS -no-obsolete -extensions h,cpp
)

get_target_property( QTPATHS_EXEC Qt6::qtpaths IMPORTED_LOCATION )
if( QTPATHS_EXEC )
	execute_process( COMMAND "${QTPATHS_EXEC}" --qt-query QT_INSTALL_TRANSLATIONS OUTPUT_VARIABLE QT_TRANSLATIONS_PATH OUTPUT_STRIP_TRAILING_WHITESPACE )
endif()

# For each language we have .ts files for, process QT's qm files
# (they need to be concatenated into one file, it's the best way to make this portable)
# Starting from Qt 6.9, we could use a built-in keyword for that :
# https://doc.qt.io/qt-6/qtlinguist-cmake-qt-add-translations.html#merging-qt-provided-translations
# starting from 6.5, another option is too use :
# or we could use https://doc.qt.io/qt-6/qt-deploy-translations.html
foreach( TRANSLATION_FILE ${TRANSLATION_FILES} )
	get_filename_component( FILENAME ${TRANSLATION_FILE} NAME )

	# Our files are named like this: CloudCompare_fr.qm
	# We want just the language part - "fr" so we can find the right files from the Qt install
	string( REGEX REPLACE "[^_]+_([^._]+).+" "\\1" LANG_CODE ${FILENAME} )
	unset( QT_LANG_FILES )
	unset( TRANSLATION_FILES_QT_EXIST )

	# These are the Qt files we want to add if they exist
	set( QT_LANG_FILES ${QT_TRANSLATIONS_PATH}/qtbase_${LANG_CODE}.qm ${QT_TRANSLATIONS_PATH}/qt_${LANG_CODE}.qm )

	# Now check that they exist
	foreach ( QT_LANG_FILE IN ITEMS ${QT_LANG_FILES} )
		if ( EXISTS ${QT_LANG_FILE} )
			list( APPEND TRANSLATION_FILES_QT_EXIST ${QT_LANG_FILE} )
		endif()
	endforeach()

	if ( TRANSLATION_FILES_QT_EXIST )
		# Finally, add a command to generate the qt language file for this language
		set( QT_LANG_FILE qt_${LANG_CODE}.qm )
		set( TRANSLATION_QM ${TRANSLATION_QM} ${CMAKE_CURRENT_BINARY_DIR}/${QT_LANG_FILE} )

		add_custom_command(
			OUTPUT
				${QT_LANG_FILE}
			DEPENDS
				${TRANSLATION_FILES_QT_EXIST}
			COMMAND
				$<TARGET_FILE:Qt6::lconvert> -o ${QT_LANG_FILE} ${TRANSLATION_FILES_QT_EXIST}
			COMMENT
				"Generating Qt language file for ${LANG_CODE}"
			VERBATIM
			)
		add_custom_target( translations_${LANG_CODE} ALL DEPENDS qt_${LANG_CODE}.qm )
	endif()
endforeach()


if( APPLE )
	install( FILES ${TRANSLATION_QM} DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/translations )
elseif( UNIX )
	install( FILES ${TRANSLATION_QM} DESTINATION share/cloudcompare/translations )
else( WIN32 )
	if( CMAKE_CONFIGURATION_TYPES )
		install( FILES ${TRANSLATION_QM} CONFIGURATIONS Release DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/translations )
		install( FILES ${TRANSLATION_QM} CONFIGURATIONS Debug DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}_debug/translations )
	else()
		install( FILES ${TRANSLATION_QM} DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/translations )
	endif()
endif()
