OSTYPE := $(shell uname -s)

SRC_FILES = \
	../Boxes/main.cpp \
	../Boxes/kbhit.cpp \
	../Boxes/signal_catch.cpp

INC_DIRS += ../Boxes

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif

DEFINES = USE_GLUT

EXE_NAME = Sample-Boxes
include ../NiteSampleMakefile

