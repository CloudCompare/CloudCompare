# -------------------------------------------------
# libqhull.pro -- Qt project for Qhull shared library
#   Built with qh_QHpointer=1
# -------------------------------------------------

include(../qhull-warn.pri)

DESTDIR = ../../lib
DLLDESTDIR = ../../bin
TEMPLATE = lib
CONFIG += shared warn_on
CONFIG -= qt

build_pass:CONFIG(debug, debug|release):{
    TARGET = qhull_d
    OBJECTS_DIR = Debug
}else:build_pass:CONFIG(release, debug|release):{
    TARGET = qhull
    OBJECTS_DIR = Release
}
win32-msvc* : QMAKE_LFLAGS += /INCREMENTAL:NO

win32-msvc* : DEF_FILE += ../../src/libqhull/qhull-exports.def

# Order object files by frequency of execution.  Small files at end.
SOURCES += rboxlib.c
SOURCES += user.c
SOURCES += global.c
SOURCES += stat.c
SOURCES += io.c
SOURCES += geom2.c
SOURCES += poly2.c
SOURCES += merge.c
SOURCES += libqhull.c
SOURCES += geom.c
SOURCES += poly.c
SOURCES += qset.c
SOURCES += mem.c
SOURCES += usermem.c
SOURCES += userprintf.c
SOURCES += userprintf_rbox.c
SOURCES += random.c

HEADERS += geom.h
HEADERS += io.h
HEADERS += libqhull.h
HEADERS += mem.h
HEADERS += merge.h
HEADERS += poly.h
HEADERS += random.h
HEADERS += qhull_a.h
HEADERS += qset.h
HEADERS += stat.h
HEADERS += user.h

OTHER_FILES += Mborland
OTHER_FILES += qh-geom.htm
OTHER_FILES += qh-globa.htm
OTHER_FILES += qh-io.htm
OTHER_FILES += qh-mem.htm
OTHER_FILES += qh-merge.htm
OTHER_FILES += qh-poly.htm
OTHER_FILES += qh-qhull.htm
OTHER_FILES += qh-set.htm
OTHER_FILES += qh-stat.htm
OTHER_FILES += qh-user.htm
