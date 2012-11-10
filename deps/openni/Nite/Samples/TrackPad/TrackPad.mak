OSTYPE := $(shell uname -s)

SRC_FILES = \
	../TrackPad/main.cpp \
	../TrackPad/kbhit.cpp \
	../TrackPad/signal_catch.cpp

INC_DIRS += ../TrackPad

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif

DEFINES = USE_GLUT

EXE_NAME = Sample-TrackPad
include ../NiteSampleMakefile

