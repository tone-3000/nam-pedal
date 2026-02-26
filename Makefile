TARGET = NAMPedal

LIBDAISY_DIR = ../../libDaisy
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core

CPP_SOURCES = \
  NAMPedal.cpp \
  nam-binary-loader/namb/get_dsp_namb.cpp \
  NeuralAmpModelerCore/NAM/activations.cpp \
  NeuralAmpModelerCore/NAM/conv1d.cpp \
  NeuralAmpModelerCore/NAM/convnet.cpp \
  NeuralAmpModelerCore/NAM/dsp.cpp \
  NeuralAmpModelerCore/NAM/get_dsp.cpp \
  NeuralAmpModelerCore/NAM/lstm.cpp \
  NeuralAmpModelerCore/NAM/ring_buffer.cpp \
  NeuralAmpModelerCore/NAM/util.cpp \
  NeuralAmpModelerCore/NAM/wavenet.cpp

C_INCLUDES = \
  -I. \
  -INeuralAmpModelerCore \
  -INeuralAmpModelerCore/Dependencies/eigen \
  -INeuralAmpModelerCore/Dependencies/nlohmann \
  -Inam-binary-loader

C_DEFS = \
  -DNAM_SAMPLE_FLOAT \
  -DNAM_USE_INLINE_GEMM \
  -D__ARM_ARCH_7EM__

USE_FATFS = 1
APP_TYPE = BOOT_QSPI
CPP_STANDARD = -std=gnu++17
OPT = -O3
LDFLAGS = -u _printf_float

include $(SYSTEM_FILES_DIR)/Makefile

CPPFLAGS += -fexceptions -ffast-math -funroll-loops -ftree-vectorize -fmove-loop-invariants
