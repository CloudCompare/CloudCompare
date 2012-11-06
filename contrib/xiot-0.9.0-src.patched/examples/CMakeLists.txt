cmake_minimum_required(VERSION 2.6)

set (VTKIMPORTER_SRC
	vtkX3DImporter.cpp
	vtkX3DNodeHandler.cpp
	vtkX3DNodeHandler.h
	vtkX3DIndexedGeometrySource.h
	vtkX3DIndexedGeometrySource.cpp
)

find_package(VTK 5.3 REQUIRED)
include(${VTK_USE_FILE})

include_directories(. ${PROJECT_SOURCE_DIR}/include ${VTK_INCLUDE_DIRS})

# vtkX3DLoader
link_directories(${VTK_LIBRARY_DIRS} )
add_executable (vtkX3DLoader ${VTKIMPORTER_SRC} vtkX3DLoader.cpp vtkX3DExporterGeneric.cxx Argument_helper.cpp)
target_link_libraries(vtkX3DLoader xiot vtkCommon vtkHybrid vtkRendering vtkIO vtkFiltering vtkCommon vtkGraphics)

# glyphTest
ADD_EXECUTABLE(x3dglyphtest Argument_helper.cc X3DTest.cxx main.cxx vtkX3DExporterGeneric.cxx)
TARGET_LINK_LIBRARIES(x3dglyphtest xiot vtkCommon vtkHybrid vtkRendering vtkIO vtkFiltering vtkCommon vtkGraphics)

# vtkX3DRewriter
add_executable (vtkX3DRewriter ${VTKIMPORTER_SRC} vtkX3DRewriter.cpp vtkX3DExporterGeneric.cxx Argument_helper.cpp)
target_link_libraries(vtkX3DRewriter xiot vtkCommon vtkHybrid vtkRendering vtkIO vtkFiltering vtkCommon vtkGraphics)

install(TARGETS vtkX3DLoader RUNTIME DESTINATION bin)
install(TARGETS x3dglyphtest RUNTIME DESTINATION bin)
