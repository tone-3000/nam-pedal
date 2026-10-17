# NAMPedal (A2-nano)

A guitar amp modeler pedal for the [PedalPCB Terrarium](https://www.pedalpcb.com/product/pcb351/) (a [Daisy Seed](https://www.electro-smith.com/daisy/daisy) platform) running Neural Amp Modeler (NAM) models of the **A2-nano** architecture. The `.namb` binary model is embedded in flash at build time, and the pedal provides input gain, output volume, and a 3-band EQ (bass/mid/treble), plus a footswitch-toggled bypass.

Inference uses hand-tuned, dependency-free C code specialized for the A2-nano WaveNet architecture and optimized for embedded Cortex-M7 targets. There is no dependency on the NeuralAmpModelerCore C++ library at build or runtime. Any A2-nano `.namb` whose weights match the fixed architecture can be swapped in at runtime.

![TONE3000 DIY Pedal and PCB (render)](docs/images/hero.jpg)

## Hardware

The `hardware/` directory has everything to build the TONE3000 DIY Pedal: KiCad project and Gerbers, bill of
materials, 1590XX enclosure drill template and faceplate artwork, 3D models and renders. See
[`hardware/README.md`](hardware/README.md) and the step-by-step [build guide](https://www.tone3000.com/guides/diy-pedal).

| | |
| --- | --- |
| [![Pedal](docs/images/pedal.jpg)](hardware/renders) | [![PCB](docs/images/pcb.jpg)](hardware/pcb) |

![Build animation](docs/images/build-animation.gif)

## Motivation

This was created as a helpful blueprint to guide the creation of embedded devices that can run NAM. For more information on the work that led to this, check [this blog post](https://www.tone3000.com/blog/running-nam-on-embedded-hardware).

## Prerequisites

- [DaisyToolchain](https://github.com/electro-smith/DaisyToolchain) — ARM cross-compiler and tools for the Daisy platform
- A PedalPCB Terrarium (Daisy Seed) board
- A USB cable and serial terminal application (see note above)

> **Important — USB serial terminal required:** The firmware currently waits
> for a USB serial connection at startup (`StartLog(true)`). If no serial
> terminal is connected, the board will **stall indefinitely** and never reach
> the audio engine. You **must** open a serial terminal (e.g. `screen`,
> `minicom`, PuTTY, or the Arduino Serial Monitor) on the Daisy's USB port
> before powering on / resetting the board. See the [Serial Monitor](#serial-monitor)
> section for details.

## Setup

### 1. Clone DaisyExamples and build libDaisy

```bash
git clone https://github.com/electro-smith/DaisyExamples
cd DaisyExamples
git submodule update --init --recursive
cd libDaisy
make
cd ..
```

### 2. Clone NAMPedal

```bash
cd seed
git clone -b a2-nano https://github.com/tone-3000/NAMPedal
cd NAMPedal
```

## Building

```bash
make
```

This cross-compiles for the STM32H750 (Cortex-M7) using `BOOT_QSPI` app type for the larger binary size NAM requires.

### Toolchain path

`make` needs `arm-none-eabi-gcc` (and friends) to be a complete toolchain —
one that bundles the newlib C library, not just the compiler binaries. If
it's missing (e.g. some package-manager builds, like Homebrew's on Linux,
ship the compiler without newlib), the very first file fails to compile with:

```
fatal error: stdint.h: No such file or directory
```

If you hit that, install a complete toolchain instead — either
[DaisyToolchain](https://github.com/electro-smith/DaisyToolchain) or the
[Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
both work.

By default the Makefile leaves `GCC_PATH` empty, which makes libDaisy's
`core/Makefile` fall back to whatever `arm-none-eabi-gcc` it finds on your
`PATH`. If your toolchain isn't on `PATH`, point at it **without editing the
Makefile** — either per invocation:

```bash
make GCC_PATH=/path/to/toolchain/bin
```

or once, for every future `make` in this shell, by adding this to your shell
profile (`~/.bashrc`, `~/.zshrc`, etc.):

```bash
export GCC_PATH=/path/to/toolchain/bin
```

## Flashing

Because this app leverages Daisy's QSPI flash memory to accommodate program size, a *bootloader* must be flashed before the main program.

To install a bootloader, put the Daisy Seed into DFU mode (hold BOOT, press RESET), then:

```bash
make program-boot
```

When the Daisy power-cycles (can trigger manually by pressing RESET), you will see the BOOT led sweep in and out, indicating a grace period during which the device can be flashed by running:

```bash
make program-dfu
```

The grace period can be extended indefinitely by pressing BOOT. 

## Usage

### Serial Monitor

The firmware calls `StartLog(true)` at boot, which **blocks until a USB serial
terminal is connected**. If you power on the board without a serial terminal
attached, it will appear to do nothing — it is waiting for the USB connection.

**Before powering on or resetting the board**, open a serial terminal on the
Daisy's USB serial port using one of the methods below.

#### macOS

The Daisy shows up as `/dev/cu.usbmodem*`. Find the exact name and connect:

```bash
ls /dev/cu.usbmodem*
screen /dev/cu.usbmodem12345 115200
```

To exit `screen`, press `Ctrl-A` then `K` and confirm with `y`.

Alternatively, if you have [Homebrew](https://brew.sh) you can install `minicom`:

```bash
brew install minicom
minicom -D /dev/cu.usbmodem12345 -b 115200
```

#### Linux

The Daisy shows up as `/dev/ttyACM*`. You may need to add your user to the
`dialout` group for access (log out and back in after):

```bash
sudo usermod -aG dialout $USER
```

Then find the port and connect:

```bash
ls /dev/ttyACM*
screen /dev/ttyACM0 115200
```

Or using `minicom`:

```bash
sudo apt install minicom          # Debian / Ubuntu
minicom -D /dev/ttyACM0 -b 115200
```

#### Windows

1. **Find the COM port:** Open Device Manager and expand **Ports (COM & LPT)**.
   The Daisy will appear as a USB Serial Device (e.g. `COM3`).

2. **Connect with PuTTY:**
   - Download [PuTTY](https://www.putty.org/)
   - Set **Connection type** to **Serial**
   - Enter the COM port (e.g. `COM3`) and speed `115200`
   - Click **Open**

3. **Or use the Arduino IDE:** Open the Serial Monitor (`Tools > Serial
   Monitor`), select the correct port, and set the baud rate to `115200`.

4. **Or use PowerShell** (no extra software needed):

   ```powershell
   # Replace COM3 with your port
   $port = New-Object System.IO.Ports.SerialPort COM3,115200
   $port.Open()
   while ($true) { if ($port.BytesToRead) { $port.ReadExisting() | Write-Host -NoNewline } }
   ```

Once the serial terminal is connected you will see boot messages, model loading
status, a one-shot benchmark, and a once-per-second diagnostics line:

```
NAMPedal (A2-nano C): booting...
Loading embedded model
  embedded model: 7516 bytes
  weights: offset=32 count=1871
  prewarming (6332 samples)...
  model ready (XX ms)
Model load: OK
Benchmark: XXXXXX cycles for 48 frames (budget=480000)
  X.XX ms (deadline 1.00 ms)
Audio engine started
cb=48  cycles=XXXXX  max=XXXXX  gain=1.00  vol=0.75  eq[0.0 0.0 0.0]  BYPASS
```

### Getting a NAM Model

The Terrarium has no SD card slot, so the model is **embedded in the firmware
and compiled into QSPI flash**. The build embeds whatever `model.namb` is
present in the project directory (via `gen_model_data.py`, which produces
`model_data.h`); swap in a different `model.namb` and rebuild to change models.
A2-nano `.nam` models are distributed as JSON, so you need to convert them to
`.namb` first. The expected payload is exactly 1871 float32 weights (7484
bytes) plus the 32-byte `.namb` header.

> **Note:** Only A2-nano models are supported. The inference code is
> specialized at build time for a single WaveNet architecture (1 layer array,
> channels=3, bottleneck=3, kernel sizes [6, 15], 23 layers, head kernel=16).
> Any other architecture will be rejected by `nam_load_weights()` at load
> time.

#### 1. Obtain an A2-nano `.nam` model

Start from any A2-nano-shaped `.nam` file you have trained or been given.
Other architectures (nano, feather, standard, etc.) are **not** compatible
with this firmware.

#### 2. Convert `.nam` to `.namb`

The converter lives in the
[nam-binary-loader](https://github.com/tone-3000/nam-binary-loader) repository
(external — not a submodule of this repo). Clone and build it somewhere
outside this project:

```bash
git clone https://github.com/tone-3000/nam-binary-loader
cd nam-binary-loader
mkdir build && cd build
cmake .. -DNAM_CORE_PATH=/path/to/NeuralAmpModelerCore
make nam2namb
```

(`nam2namb` itself depends on NeuralAmpModelerCore to parse the JSON — point
`NAM_CORE_PATH` at a local checkout of that repo.) Then convert your model:

```bash
./nam2namb /path/to/your-a2-nano-model.nam model.namb
```

#### 3. Place `model.namb` and rebuild

Copy the resulting `model.namb` into the project directory (next to the
`Makefile`), replacing the existing one, then rebuild and reflash. The build
regenerates `model_data.h` and compiles the model into flash automatically.

### Running the Pedal

1. Build and flash the firmware (with your `model.namb` embedded — see above)
2. Connect the Daisy Seed to your computer via USB and open a serial terminal (see above)
3. Power on or reset the board — the serial terminal must be open **before** this step
4. Wait for the `Audio engine started` message in the serial output
5. Connect your guitar to the audio input and an amp/headphones to the audio output

### Controls

| Control       | Function                          |
|---------------|-----------------------------------|
| KNOB 1        | Input gain (noon ≈ unity)         |
| KNOB 2        | Output volume                     |
| KNOB 3        | (unused)                          |
| KNOB 4        | Bass (±12 dB, noon = flat)        |
| KNOB 5        | Mid (±12 dB, noon = flat)         |
| KNOB 6        | Treble (±12 dB, noon = flat)      |
| FOOTSWITCH 1  | Toggle bypass                     |

Signal chain: input gain → NAM model → 3-band EQ → output volume.

### LED Indicator

- **LED 1 off** — Effect is bypassed (clean passthrough)
- **LED 1 on** — Effect is active (NAM model + EQ processing)

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Board appears dead after flashing | No serial terminal connected; firmware is waiting for USB | Connect a serial terminal, then reset the board |
| `Model load: FAILED` with `bad magic` | `model.namb` is not a valid `.namb` file | Regenerate `model.namb` with `nam2namb` and rebuild |
| `nam_load_weights failed` | `.namb` weight count is not 1871 | Embed an A2-nano `model.namb` (exactly 1871 weights) and rebuild |
| `nam_load_weights failed (not an A2-nano model?)` | `.namb` weight count is not 1871 | Regenerate the `.namb` from an A2-nano `.nam` model |
| Audio glitches / dropouts | Model inference exceeds the 1 ms per-block deadline | Confirm the firmware was built with `-O2` and the DaisyToolchain; check the startup benchmark output |

## How it Works

- Audio is processed mono — the left input channel is fed through the NAM model and the output is duplicated to both stereo channels
- The model is loaded once at startup from the flash-embedded `.namb`: `nam_init()` initialises state, `nam_load_weights()` ingests the 1871 float32 weights from the `.namb` payload, and the prewarm loop pushes 6332 samples of silence through to fill the WaveNet receptive field
- Inference runs in hand-tuned C specialized for the A2-nano architecture: fused sample-at-a-time layers, power-of-2 circular ring buffers, unrolled operations for specific kernel sizes. Weights and the frequently-accessed small ring buffers live in DTCM; the largest ring buffers (dilation=239) live in AXI SRAM with anti-aliasing padding between them
- The audio block size is 48 samples at 48 kHz, giving a 1 ms processing deadline per block
- FPU Flush-to-Zero and Default-NaN modes are enabled to avoid costly denormal handling on the Cortex-M7
