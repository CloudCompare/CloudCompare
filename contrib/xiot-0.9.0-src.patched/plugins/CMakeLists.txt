cmake_minimum_required(VERSION 2.6)

set (X3DIMPORTER_SRC
../examples/vtkX3DImporter.cpp
../examples/vtkX3DNodeHandler.cpp
../examples/vtkX3DNodeHandler.h
../examples/vtkX3DIndexedFaceSetSource.h
../examples/vtkX3DIndexedFaceSetSource.cpp
../examples/vtkX3DIndexedLineSetSource.h
../examples/vtkX3DIndexedLineSetSource.cpp
)

include_directories(. ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/examples)

FIND_PACKAGE(ParaView REQUIRED)
INCLUDE(${PARAVIEW_USE_FILE})

ADD_PARAVIEW_PLUGIN(pvX3DReader "1.0" 
   SERVER_MANAGER_XML X3DReader.xml
   SERVER_MANAGER_SOURCES vtkX3DSource.cxx 
   SERVER_SOURCES ${X3DIMPORTER_SRC}
   GUI_RESOURCE_FILES X3DReaderGUI.xml)


target_link_libraries(pvX3DReader xiot)
