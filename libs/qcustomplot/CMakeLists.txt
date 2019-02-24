PROJECT(qcustomplot)

set( QCUSTOMPLOT_HEADERS qcustomplot.h )
set( QCUSTOMPLOT_SOURCES qcustomplot.cpp )

add_library(${PROJECT_NAME} STATIC ${QCUSTOMPLOT_HEADERS} ${QCUSTOMPLOT_SOURCES} )
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "libs") 