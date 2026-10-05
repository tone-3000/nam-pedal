TARGET = NAMPedal

# Empty by default: libDaisy's core Makefile falls back to plain
# arm-none-eabi-gcc/g++/etc. resolved from PATH when GCC_PATH is unset.
# If your toolchain isn't on PATH, point at it without editing this file:
#   make GCC_PATH=/path/to/toolchain/bin
#   export GCC_PATH=/path/to/toolchain/bin   (in your shell profile)
GCC_PATH ?= 

LIBDAISY_DIR = ../../libDaisy
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core

CPP_SOURCES = NAMPedal.cpp
C_SOURCES = nam_model.c

C_INCLUDES = -I.

APP_TYPE = BOOT_QSPI
CPP_STANDARD = -std=gnu++17
OPT = -O2
LDFLAGS = -u _printf_float

include $(SYSTEM_FILES_DIR)/Makefile

# Models are stored in a separate QSPI flash region (0x90800000) and loaded at
# runtime — no rebuild needed to swap models.  Use pack_models.py to build and
# flash a model bank independently of this firmware.

# Daisy audio block size is 48 frames — must match NAMPedal.cpp SetAudioBlockSize.
# NAM_DTCM places weights and small ring buffers in fast DTCM (Cortex-M7).
ifeq ($(LOGGING),1)
CPPFLAGS += -DLOGGING
CFLAGS   += -DLOGGING
endif

CFLAGS   += -ffast-math -fno-unroll-loops -ftree-vectorize \
            -DNAM_MAX_BUFFER_SIZE=48 \
            -DNAM_DTCM='__attribute__((section(".dtcmram_bss")))'
CPPFLAGS += -DNAM_MAX_BUFFER_SIZE=48 \
            -DNAM_DTCM='__attribute__((section(".dtcmram_bss")))'

# Override program-dfu to support automatic DFU trigger via the pedal's USB
# CDC serial port.  The firmware responds to a 'D' byte by resetting into DFU
# mode.  Requires pyserial (pip install pyserial).
#
# Usage:
#   make program-dfu PORT=/dev/cu.usbmodemXXXX   # trigger DFU automatically
#   make program-dfu                               # Daisy already in DFU mode
#
# On macOS use /dev/cu.usbmodem* (callout), NOT /dev/tty.usbmodem*.
# The tty. device blocks on open waiting for carrier detect; cu. does not.
program-dfu:
	@echo "program-dfu: PORT=[$(PORT)]"
ifdef PORT
	@echo "Sending DFU trigger to $(PORT)..."
	python3 -c "\
import serial, time; \
s = serial.Serial('$(PORT)', baudrate=115200, timeout=1); \
s.dtr = True; \
time.sleep(0.2); s.write(b'D'); s.flush(); time.sleep(0.5); s.close(); \
print('DFU trigger sent to $(PORT), waiting for re-enumeration...')"
	sleep 4
else
	@echo "(no PORT set — device should already be in DFU mode)"
endif
	dfu-util -a 0 -s $(FLASH_ADDRESS):leave -D $(BUILD_DIR)/$(TARGET_BIN) -d ,0483:$(USBPID)
