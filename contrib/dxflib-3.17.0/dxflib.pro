exists(../../../shared.pri) {
    include( ../../../shared.pri )
}

win32-msvc {
  DEFINES += _CRT_SECURE_NO_DEPRECATE
}

INCLUDEPATH += src
HEADERS = \
    src/dl_attributes.h \
    src/dl_codes.h \
    src/dl_creationadapter.h \
    src/dl_creationinterface.h \
    src/dl_dxf.h \
    src/dl_entities.h \
    src/dl_exception.h \
    src/dl_extrusion.h \
    src/dl_writer.h \
    src/dl_writer_ascii.h

SOURCES = \
    src/dl_dxf.cpp \
    src/dl_writer_ascii.cpp

TARGET = dxflib
TEMPLATE = lib
CONFIG += staticlib
DEFINES += DXFLIB_LIBRARY
