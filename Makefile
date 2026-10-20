TARGET = t3k_pedal

# Empty by default: libDaisy's core Makefile falls back to plain
# arm-none-eabi-gcc/g++/etc. resolved from PATH when GCC_PATH is unset.
# If your toolchain isn't on PATH, point at it without editing this file:
#   make GCC_PATH=/path/to/toolchain/bin
#   export GCC_PATH=/path/to/toolchain/bin   (in your shell profile)
GCC_PATH ?= 

# libDaisy (git submodule pinned to a release; v9.0.0+ knows the Daisy Seed3).
# The static library is built on demand below, so `make` is enough after
#   git submodule update --init --recursive
# Override to build against a libDaisy checkout elsewhere:
#   make LIBDAISY_DIR=/path/to/libDaisy
LIBDAISY_DIR ?= libDaisy
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core

# NAM A2 engines (tone-3000/embedded, git submodule pinned to a release).
# The A2-Lite Cortex-M7 engine and the .namb reader are built from here.
EMBEDDED_DIR = embedded
A2LITE_DIR   = $(EMBEDDED_DIR)/engines/cortex-m7/a2lite
NAMB_DIR     = $(EMBEDDED_DIR)/namb

CPP_SOURCES = t3k_pedal.cpp
C_SOURCES = $(A2LITE_DIR)/nam_a2lite.c $(NAMB_DIR)/namb_reader.c

C_INCLUDES = -I. -I$(A2LITE_DIR) -I$(NAMB_DIR)

APP_TYPE = BOOT_QSPI
CPP_STANDARD = -std=gnu++17
OPT = -O2
LDFLAGS = -u _printf_float

# libDaisy's QSPI linker script with the engine repo's itcm_section.lds
# inserted before .text, so the engine's code runs from zero-wait ITCM
# (t3k_pedal.cpp copies it from QSPI at boot). Generated below, with
# daisyConvol's engine and the CMSIS FFT kernels added to the same section:
# run from QSPI through the I-cache next to NAM, they cost about twice as much.
LDSCRIPT = $(BUILD_DIR)/$(TARGET).lds

ifeq ($(wildcard $(SYSTEM_FILES_DIR)/Makefile),)
$(error libDaisy not found at $(LIBDAISY_DIR): run 'git submodule update --init --recursive' (or pass LIBDAISY_DIR=...))
endif

# Cabinet IR convolution: daisyConvol (VZtec/daisyConvol, git submodule pinned
# to a release), zero-latency partitioned FFT convolution on CMSIS-DSP.
# Its .mk adds the engine and CMSIS FFT sources, then includes libDaisy's core
# Makefile itself.  The engine runs 16-sample blocks (the only power of two
# dividing the 48-frame audio block; t3k_pedal.cpp calls it three times per
# callback), and IRs up to 4096 samples need no SDRAM or main-loop stage.
# DAISYCONVOL_MAX_IR must match kMaxIrLen in t3k_pedal.cpp.
DAISYCONVOL_DIR    = daisyConvol
DAISYCONVOL_BLOCKS = 16
DAISYCONVOL_MAX_IR = 4096
ifeq ($(wildcard $(DAISYCONVOL_DIR)/daisyconvol.mk),)
$(error daisyConvol not found at $(DAISYCONVOL_DIR): run 'git submodule update --init --recursive')
endif
include $(DAISYCONVOL_DIR)/daisyconvol.mk

# Presets are stored in a separate QSPI flash region (0x90600000) and loaded
# at runtime — no rebuild needed to swap models.  Use the companion app (app/)
# or pack_models.py to build and flash a preset bank independently of this
# firmware.

# LOGGING=1: boot log, a one-shot cycle-count benchmark and a once-per-second
# status line over USB serial (115200 baud).
ifeq ($(LOGGING),1)
CPPFLAGS += -DLOGGING
CFLAGS   += -DLOGGING
endif

# Daisy audio block size is 48 frames — must match t3k_pedal.cpp SetAudioBlockSize.
# NAM_A2LITE_MAX_BUFFER_SIZE sizes the engine's structs; it must be the same in
# every file that includes nam_a2lite.h.  NAM_DTCM places t3k_pedal.cpp's own
# hot buffers (and the engine instances + captures) in fast DTCM (Cortex-M7).
NAM_DEFS = -DNAM_A2LITE_MAX_BUFFER_SIZE=48 \
           -DNAM_DTCM='__attribute__((section(".dtcmram_bss")))'
CFLAGS   += -ffast-math $(NAM_DEFS)
CPPFLAGS += $(NAM_DEFS)

# libDaisy's CPPFLAGS expands CFLAGS, so -ffast-math reaches C++ too.  Keep
# daisyConvol's engine IEEE-strict: its NaN/inf guards on the IR and the input
# (fabs(x) <= FLT_MAX) would be folded away under -ffinite-math-only.
$(BUILD_DIR)/conv_engine.o: CPPFLAGS += -fno-fast-math

$(BUILD_DIR)/nam_a2lite.o $(BUILD_DIR)/t3k_pedal.o: $(A2LITE_DIR)/nam_a2lite.h

ITCM_LDS = $(EMBEDDED_DIR)/bench/cortex-m7/daisy_seed/itcm_section.lds
ITCM_EXTRA_OBJS = conv_engine fft_backend_cmsis arm_cfft_f32 arm_cfft_radix8_f32 \
                  arm_rfft_fast_f32 cmsis_bitreversal
$(BUILD_DIR)/$(TARGET).lds: $(SYSTEM_FILES_DIR)/STM32H750IB_qspi.lds $(ITCM_LDS) Makefile | $(BUILD_DIR)
	awk -v extra='$(ITCM_EXTRA_OBJS)' \
	    'FNR == NR { blk = blk $$0 "\n"; \
	                 if ($$0 ~ /nam_a2lite\*\.o/) { n = split(extra, o, " "); \
	                   for (i = 1; i <= n; i++) blk = blk "\t\t*" o[i] ".o(.text .text*)\n" } \
	                 next } \
	     !done && /^[ \t]*\.text[ \t]*:/ { printf "%s", blk; done = 1 } \
	     { print } \
	     END { if (!done) { print "no .text section found" > "/dev/stderr"; exit 1 } }' \
	    $(ITCM_LDS) $< > $@

$(BUILD_DIR)/$(TARGET).elf: $(LDSCRIPT) $(LIBDAISY_DIR)/build/libdaisy.a

# Build libDaisy once if its library is missing.
$(LIBDAISY_DIR)/build/libdaisy.a:
	$(MAKE) -C $(LIBDAISY_DIR) GCC_PATH=$(GCC_PATH)

libdaisy:
	$(MAKE) -C $(LIBDAISY_DIR) GCC_PATH=$(GCC_PATH)

clean-all: clean
	$(MAKE) -C $(LIBDAISY_DIR) clean

.PHONY: libdaisy clean-all

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
