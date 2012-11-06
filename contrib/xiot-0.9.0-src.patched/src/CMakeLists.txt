

set(XIOT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

# Find the Z library
if (WIN32)
	if (ZLIB_ROOT)
		set (CMAKE_PREFIX_PATH ${ZLIB_ROOT})
	endif()
endif()
		
find_package(ZLIB)
if ( NOT ZLIB_FOUND )
	set(ZLIB_ROOT CACHE PATH "Location of the zlib library")
	message(FATAL_ERROR "Unable to find zlib library. Please set the the ZLIB_ROOT to point to the root of the zlib install directory." )
endif()
#if (WIN32)
  # Hint to included zlib resources
  #set(ZLIB_ROOT ${PROJECT_SOURCE_DIR}/contrib/zlib)
  #set(ZLIB_INCLUDE_DIR ${ZLIB_ROOT}/include CACHE PATH "Include path of zlib")
  #set(ZLIB_LIBRARY ${ZLIB_ROOT}/${WIN_ARCH}/lib/zlib.lib CACHE FILEPATH  "z library" )
#endif (WIN32)
#message(${ZLIB_INCLUDE_DIR})
#message(${ZLIB_LIBRARIES})

set(EXPAT_PARSER_IMPL_SRC X3DXMLLoaderExpatImpl.cpp X3DXMLAttributesExpatImpl.cpp)
	
# Always use Xerces

set(Xerces_USE_STATIC_LIBS ON)
find_package( Xerces QUIET )
if ( NOT Xerces_FOUND )
	set(XERCES_ROOT CACHE PATH "Location of the xerces library")
	message(FATAL_ERROR "Unable to find xerces library. Please set the the XERCES_ROOT to point to the root of the xerces directory." )
endif()

set(XML_PARSER_IMPL_SRC	X3DXMLLoaderXercesImpl.cpp X3DXMLAttributesXercesImpl.cpp)

set(XIOT_HEADER
	${XIOT_INCLUDE_DIR}/xiot/X3DTypes.h
	${XIOT_INCLUDE_DIR}/xiot/X3DLoader.h
	${XIOT_INCLUDE_DIR}/xiot/X3DXMLLoader.h
	${XIOT_INCLUDE_DIR}/xiot/X3DAttributes.h
	${XIOT_INCLUDE_DIR}/xiot/X3DXMLAttributes.h
	${XIOT_INCLUDE_DIR}/xiot/X3DFIAttributes.h
	${XIOT_INCLUDE_DIR}/xiot/X3DSwitch.h
	${XIOT_INCLUDE_DIR}/xiot/X3DNodeHandler.h
	${XIOT_INCLUDE_DIR}/xiot/X3DDefaultNodeHandler.h
	${XIOT_INCLUDE_DIR}/xiot/X3DFILoader.h
	${XIOT_INCLUDE_DIR}/xiot/X3DDataTypeFactory.h
	${XIOT_INCLUDE_DIR}/xiot/X3DParseException.h
	${XIOT_INCLUDE_DIR}/xiot/X3DFICompressionTools.h
	${XIOT_INCLUDE_DIR}/xiot/X3DParserVocabulary.h
	${XIOT_INCLUDE_DIR}/xiot/X3DFIEncodingAlgorithms.h
	${XIOT_INCLUDE_DIR}/xiot/X3DWriter.h
	${XIOT_INCLUDE_DIR}/xiot/X3DFIEncoder.h
	${XIOT_INCLUDE_DIR}/xiot/X3DWriterFI.h
	${XIOT_INCLUDE_DIR}/xiot/X3DWriterXML.h
)


set(XIOT_SRC
	X3DTypes.cpp
	X3DLoader.cpp
	X3DAttributes.cpp
	X3DFIAttributes.cpp
	X3DSwitch.cpp
	X3DDefaultNodeHandler.cpp
	X3DFILoader.cpp
	X3DDataTypeFactory.cpp
	X3DParseException.cpp
	X3DFICompressionTools.cpp
	X3DParserVocabulary.cpp
	X3DFIEncodingAlgorithms.cpp
	X3DWriterFI.cpp
	X3DWriter.cpp
	X3DFIEncoder.cpp
	X3DWriterXML.cpp
)

set(OPENFI_SRC
	FIEncoder.cpp
	FIDecoder.cpp
	FIContentHandler.cpp
	FISAXParser.cpp
	FIParserVocabulary.cpp
	FIEncodingAlgorithms.cpp
)

set(OPENFI_HEADER
	${XIOT_INCLUDE_DIR}/xiot/FIConstants.h
	${XIOT_INCLUDE_DIR}/xiot/FITypes.h
	${XIOT_INCLUDE_DIR}/xiot/FIDecoder.h
	${XIOT_INCLUDE_DIR}/xiot/FIDecoder.h
	${XIOT_INCLUDE_DIR}/xiot/FIEncoder.h
	${XIOT_INCLUDE_DIR}/xiot/FIContentHandler.h
	${XIOT_INCLUDE_DIR}/xiot/FISAXParser.h
	${XIOT_INCLUDE_DIR}/xiot/FIParserVocabulary.h
	${XIOT_INCLUDE_DIR}/xiot/FIEncodingAlgorithms.h
)

# set up the include directories
include_directories( ${XIOT_INCLUDE_DIR} )
include_directories( ${ZLIB_INCLUDE_DIR} )
include_directories( ${Xerces_INCLUDE_DIR} )

# set up openFI
add_library(openFI SHARED ${OPENFI_SRC} ${OPENFI_HEADER})

# set up XIOT
add_library(xiot SHARED ${XIOT_SRC} ${XIOT_HEADER} ${XML_PARSER_IMPL_SRC} ${XIOT_INCLUDE_DIR}/xiot/XIOTConfig.h)
target_link_libraries (xiot debug ${Xerces_LIBRARY_DEBUG} optimized ${Xerces_LIBRARY_RELEASE}) 
set_target_properties( xiot PROPERTIES COMPILE_DEFINITIONS XERCES_STATIC_LIBRARY )
if (MSVC)
	set_target_properties( xiot PROPERTIES COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS )
endif()
target_link_libraries (xiot ${ZLIB_LIBRARIES}) 
target_link_libraries (xiot openFI) 


add_executable(sourceGenerator gen/sourceGenerator.cpp)
target_link_libraries (sourceGenerator xiot) 

##
# install stuff
##
install(TARGETS xiot openFI
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
install(FILES ${XIOT_HEADER} ${OPENFI_HEADER} ${XIOT_INCLUDE_DIR}/xiot/XIOTConfig.h ${XIOT_INCLUDE_DIR}/xiot/FIConfig.h DESTINATION include/xiot)

##
# vtk distribution stuff
##
option(EXPORT_VTK_DIST "Build a XIOT distribution for VTK" OFF)
mark_as_advanced(EXPORT_VTK_DIST)

if(EXPORT_VTK_DIST)
add_custom_target(GENERATE_VTK_DIST DEPENDS ${XIOT_SRC} ${XIOT_HEADER} ${OPENFI_SRC} ${OPENFI_HEADER})
add_custom_command(TARGET GENERATE_VTK_DIST
                     PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} ARGS -E make_directory ${PROJECT_BINARY_DIR}/vtkxiot/include/xiot
                  )

foreach(SFILE ${XIOT_SRC} ${OPENFI_SRC} ${EXPAT_PARSER_IMPL_SRC})
  set(SFILE ${PROJECT_SOURCE_DIR}/src/${SFILE})
  add_custom_command(TARGET GENERATE_VTK_DIST
                     PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} ARGS -E copy ${SFILE} ${PROJECT_BINARY_DIR}/vtkxiot
					)
endforeach(SFILE)

foreach(SFILE ${XIOT_HEADER} ${OPENFI_HEADER})
  add_custom_command(TARGET GENERATE_VTK_DIST
                     PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} ARGS -E copy ${SFILE} ${PROJECT_BINARY_DIR}/vtkxiot/include/xiot
					)
endforeach(SFILE)

set(VTK_DIST_SRC_LIST "${XIOT_SRC};${OPENFI_SRC};${EXPAT_PARSER_IMPL_SRC}")
string(REPLACE ";" "\n\t" VTK_DIST_SRC "${VTK_DIST_SRC_LIST}")
configure_file( conf/vtkCMakeLists.txt.in ${PROJECT_BINARY_DIR}/vtkxiot/CMakeLists.txt @ONLY)
configure_file( conf/vtkXIOTConfig.h.in ${PROJECT_BINARY_DIR}/vtkxiot/include/xiot/XIOTConfig.h.in COPYONLY)
configure_file( conf/vtkFIConfig.h ${PROJECT_BINARY_DIR}/vtkxiot/include/xiot/FIConfig.h COPYONLY)

endif(EXPORT_VTK_DIST)