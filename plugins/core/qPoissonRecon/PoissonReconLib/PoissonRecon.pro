QT       -= core gui
INCLUDEPATH -= .

### begin mac specific part #################
# On OSX xcode clang does NOT support OpenMP. 
# Use these two lines if you installed an alternative clang with macport 
# (something like 'sudo port install clang-3.9') 
macx:QMAKE_CXX = clang++-mp-3.9
macx:QMAKE_LFLAGS += -L/opt/local/lib/libomp -lomp

# Use this if you want to use the standard clang distributed with xcode
# macx:QMAKE_CXXFLAGS-= -fopenmp

# Mac specific Config required to avoid to make application bundles
CONFIG -= app_bundle
### end of mac specific part #################

QMAKE_CXXFLAGS+=-fopenmp -Wsign-compare -O3 -DRELEASE -funroll-loops -ffast-math

CONFIG += console warn_off
TEMPLATE = app

SOURCES +=  Src/PoissonRecon.cpp \
            Src/MarchingCubes.cpp \
            Src/PlyFile.cpp \
            Src/CmdLineParser.cpp \
            Src/Factor.cpp \
            Src/Geometry.cpp


TARGET=PoissonRecon

