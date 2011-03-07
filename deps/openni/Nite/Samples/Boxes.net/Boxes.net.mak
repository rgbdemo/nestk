BIN_DIR = ../Bin

NETEXE_NAME = Sample-Boxes.net

SRC_FILES = \
	../Boxes.net/*.cs \
	../Res/AssemblyInfo-NITE.cs

USED_LIBS += OpenNI.net
CSFLAGS = -unsafe
NET_WIN_FORMS = 1

include ../../CommonMakefile

