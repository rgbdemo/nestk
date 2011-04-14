OSTYPE := $(shell uname -s)

SRC_FILES = \
	../PointViewer/main.cpp \
	../PointViewer/PointDrawer.cpp

INC_DIRS += ../PointViewer

EXE_NAME = Sample-PointViewer

DEFINES = USE_GLUT

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif

include ../NiteSampleMakefile

