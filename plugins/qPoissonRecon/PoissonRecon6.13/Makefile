# /********************************************************************************************
# * File:		Makefile
# * Author:		$LastChangedBy: matthew $
# * Revision:	$Revision: 233 $
# * Last Updated:	$LastChangedDate: 2006-11-10 15:03:28 -0500 (Fri, 10 Nov 2006) $
# ********************************************************************************************/

PR_TARGET=PoissonRecon
ST_TARGET=SurfaceTrimmer
PR_SOURCE=CmdLineParser.cpp Factor.cpp Geometry.cpp MarchingCubes.cpp PlyFile.cpp PoissonRecon.cpp
ST_SOURCE=CmdLineParser.cpp Factor.cpp Geometry.cpp MarchingCubes.cpp PlyFile.cpp SurfaceTrimmer.cpp

CFLAGS += -fopenmp -Wno-deprecated
LFLAGS += -lgomp

CFLAGS_DEBUG = -DDEBUG -g3
LFLAGS_DEBUG =

CFLAGS_RELEASE = -O3 -DRELEASE -funroll-loops -ffast-math
LFLAGS_RELEASE = -O3 

SRC = Src/
BIN = Bin/Linux/
INCLUDE = /usr/include/

CC=gcc
CXX=g++
MD=mkdir

PR_OBJECTS=$(addprefix $(BIN), $(addsuffix .o, $(basename $(PR_SOURCE))))
ST_OBJECTS=$(addprefix $(BIN), $(addsuffix .o, $(basename $(ST_SOURCE))))


all: CFLAGS += $(CFLAGS_DEBUG)
all: LFLAGS += $(LFLAGS_DEBUG)
all: $(BIN)
all: $(BIN)$(PR_TARGET)
all: $(BIN)$(ST_TARGET)

release: CFLAGS += $(CFLAGS_RELEASE)
release: LFLAGS += $(LFLAGS_RELEASE)
release: $(BIN)
release: $(BIN)$(PR_TARGET)
release: $(BIN)$(ST_TARGET)

clean:
	rm -f $(BIN)$(PR_TARGET)
	rm -f $(BIN)$(ST_TARGET)
	rm -f $(PR_OBJECTS)
	rm -f $(ST_OBJECTS)

$(BIN):
	$(MD) -p $(BIN)

$(BIN)$(PR_TARGET): $(PR_OBJECTS)
	$(CXX) -o $@ $(PR_OBJECTS) $(LFLAGS)

$(BIN)$(ST_TARGET): $(ST_OBJECTS)
	$(CXX) -o $@ $(ST_OBJECTS) $(LFLAGS)

$(BIN)%.o: $(SRC)%.c
	mkdir -p $(BIN)
	$(CC) -c -o $@ $(CFLAGS) -I$(INCLUDE) $<

$(BIN)%.o: $(SRC)%.cpp
	mkdir -p $(BIN)
	$(CXX) -c -o $@ $(CFLAGS) -I$(INCLUDE) $<

