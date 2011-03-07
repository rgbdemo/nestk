OSTYPE := $(shell uname -s)

SRC_FILES = \
	../CircleControl/main.cpp \
	../CircleControl/kbhit.cpp \
	../CircleControl/signal_catch.cpp

INC_DIRS += ../CircleControl

DEFINES = USE_GLUT

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif

EXE_NAME = Sample-CircleControl
include ../NiteSampleMakefile

