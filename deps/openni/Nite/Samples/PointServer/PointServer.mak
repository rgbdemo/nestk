
SRC_FILES = ../PointServer/main.cpp ../PointServer/signal_catch.cpp ../PointServer/kbhit.cpp

EXE_NAME = Sample-PointServer

ifndef TARGETFS
	TARGETFS=/
endif
 
#LDFLAGS += -B$(TARGETFS)/usr/lib -Wl,--unresolved-symbols=ignore-in-shared-libs
#INC_DIRS += $(PSDK)/Include ../../Include
#LIB_DIRS += $(PSDK)/Platform/Linux-x86/Bin/Release
#BIN_DIR = ../../Bin

include ../NiteSampleMakefile
