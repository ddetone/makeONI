include ./Common/CommonDefs.mak

BIN_DIR = ./Bin

INC_DIRS = ./Include

SRC_FILES = ./Src/*.cpp

EXE_NAME = Sample-NiRecordSynthetic
USED_LIBS = OpenNI opencv_highgui opencv_core boost_filesystem boost_system

include ./Common/CommonCppMakefile

