SRC_FILES = ../SingleControl/main.cpp ../SingleControl/signal_catch.cpp ../SingleControl/kbhit.cpp

EXE_NAME = Sample-SingleControl

ifndef TARGETFS
	TARGETFS=/
endif
 
#LDFLAGS += -B$(TARGETFS)/usr/lib -Wl,--unresolved-symbols=ignore-in-shared-libs
#INC_DIRS += $(PSDK)/Include ../../Include
#LIB_DIRS += $(PSDK)/Platform/Linux-x86/Bin/Release
#BIN_DIR = ../../Bin

include ../NiteSampleMakefile
