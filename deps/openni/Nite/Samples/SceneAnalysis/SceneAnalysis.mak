OSTYPE := $(shell uname -s)

SRC_FILES = \
	../SceneAnalysis/main.cpp \
	../SceneAnalysis/SceneDrawer.cpp \
	../SceneAnalysis/kbhit.cpp \
	../SceneAnalysis/signal_catch.cpp

INC_DIRS += ../SceneAnalysis

EXE_NAME = Sample-SceneAnalysis

DEFINES = USE_GLUT

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif

include ../NiteSampleMakefile

