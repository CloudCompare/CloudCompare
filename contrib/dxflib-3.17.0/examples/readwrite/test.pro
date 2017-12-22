#include( ../../../shared.pri )

win32-msvc {
  DEFINES += _CRT_SECURE_NO_DEPRECATE
}

OBJECTS_DIR=.obj
CONFIG -= app_bundle

#macx {
#  QMAKE_MAC_SDK=/Developer/SDKs/MacOSX10.5.sdk
#  CONFIG+=x86 ppc
#}

INCLUDEPATH += ../../src
HEADERS = \
    test_creationclass.h \
    ../../src/dl_attributes.h \
    ../../src/dl_codes.h \
    ../../src/dl_creationadapter.h \
    ../../src/dl_creationinterface.h \
    ../../src/dl_dxf.h \
    ../../src/dl_entities.h \
    ../../src/dl_exception.h \
    ../../src/dl_extrusion.h \
    ../../src/dl_writer.h \
    ../../src/dl_writer_ascii.h

SOURCES = \
    main.cpp \
    test_creationclass.cpp \
    ../../src/dl_dxf.cpp \
    ../../src/dl_writer_ascii.cpp

TARGET = readwrite
TEMPLATE = app
