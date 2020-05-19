win32-msvc {
  DEFINES += _CRT_SECURE_NO_DEPRECATE
}
macx-clang* {
    exists(/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.11.sdk) {
        QMAKE_MAC_SDK = macosx10.11
    }
}

OBJECTS_DIR=.obj
CONFIG -= app_bundle

INCLUDEPATH += ../../src
HEADERS = \
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
    ../../src/dl_dxf.cpp \
    ../../src/dl_writer_ascii.cpp

TARGET = writedimension
TEMPLATE = app
