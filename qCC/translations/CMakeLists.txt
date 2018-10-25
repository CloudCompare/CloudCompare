set_directory_properties( PROPERTIES CLEAN_NO_CUSTOM 1 )

find_package( Qt5LinguistTools REQUIRED )

# Find our main Qt directory by removing directories (I don't see another way to get this)
get_filename_component( QT_PATH ${Qt5LinguistTools_DIR} PATH )
get_filename_component( QT_PATH ${QT_PATH} PATH )
get_filename_component( QT_PATH ${QT_PATH} PATH )

# find lconvert
find_program( QT_LCONVERT_EXE NAMES lconvert-qt5 lconvert PATHS
	${QT_PATH}/bin
	NO_DEFAULT_PATH
	)

if( NOT QT_LCONVERT_EXE )
	message(FATAL_ERROR "Could NOT find lconvert")
endif()

# Now handle out own .ts files
file( GLOB TRANSLATION_FILES ${CMAKE_CURRENT_LIST_DIR}/*.ts )

qt5_add_translation( TRANSLATION_QM ${TRANSLATION_FILES} )
add_custom_target( translations ALL DEPENDS ${TRANSLATION_QM} )

# For each language we have .ts files for, process Qt's qm files
# (they need to be concatenated into one file)
foreach( TRANSLATION_FILE ${TRANSLATION_FILES} )
	get_filename_component( FILENAME ${TRANSLATION_FILE} NAME )
	
	# Our files are named like this: CloudCompare_fr.qm
	# We want just the language part - "fr" so we can find the right files from the Qt install
	string( REGEX REPLACE "[^_]+_([^._]+).+" "\\1" LANG_CODE ${FILENAME} )
	
	unset( QT_LANG_FILES )
	unset( TRANSLATION_FILES_QT_EXIST )
	
	# These are the Qt files we want to add if they exist	
	set( QT_LANG_FILES 
		${QT_PATH}/translations/qtbase_${LANG_CODE}.qm
		)
		
	# Now check that they exist	
	foreach ( QT_LANG_FILE IN ITEMS ${QT_LANG_FILES} )
		if ( EXISTS ${QT_LANG_FILE} )
			list( APPEND TRANSLATION_FILES_QT_EXIST ${QT_LANG_FILE} )
		endif()
	endforeach()
	
	# Finally, add a command to generate the qt language file for this language	
	add_custom_command(
		OUTPUT
			qt_${LANG_CODE}.qm
		DEPENDS
			${TRANSLATION_FILES_QT_EXIST}
		COMMAND
			${QT_LCONVERT_EXE} -o qt_${LANG_CODE}.qm ${TRANSLATION_FILES_QT_EXIST}
		COMMENT
			"Generating Qt language file for ${LANG_CODE}"
		VERBATIM
		)
	
	add_custom_target( translations_${LANG_CODE} ALL DEPENDS qt_${LANG_CODE}.qm )
endforeach()

if( APPLE )
	install( FILES ${TRANSLATION_QM} DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Resources/translations )
elseif( UNIX )
	install( FILES ${TRANSLATION_QM} DESTINATION share/cloudcompare/translations )
else( WIN32 )
	install( FILES ${TRANSLATION_QM} DESTINATION translations )
endif()
