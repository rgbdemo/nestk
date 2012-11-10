OSTYPE := $(shell uname -s)

SRC_FILES = \
	../Players/main.cpp \
	../Players/SceneDrawer.cpp \
	../Players/kbhit.cpp \
	../Players/signal_catch.cpp

INC_DIRS += ../Players

EXE_NAME = Sample-Players

DEFINES = USE_GLUT

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif

include ../NiteSampleMakefile

