# NAMPedal

A guitar amp modeler pedal for the [Daisy Pod](https://www.electro-smith.com/daisy/pod) running [Neural Amp Modeler](https://github.com/sdatkinson/NeuralAmpModelerCore) (NAM) models. It loads `.namb` binary model files from an SD card and provides two knobs for gain (input level) and volume (output level).

> **Important — USB serial terminal required:** The firmware currently waits
> for a USB serial connection at startup (`StartLog(true)`). If no serial
> terminal is connected, the board will **stall indefinitely** and never reach
> the audio engine. You **must** open a serial terminal (e.g. `screen`,
> `minicom`, PuTTY, or the Arduino Serial Monitor) on the Daisy's USB port
> before powering on / resetting the board. See the [Serial Monitor](#serial-monitor)
> section for details.

## Prerequisites

- [DaisyToolchain](https://github.com/electro-smith/DaisyToolchain) — ARM cross-compiler and tools for the Daisy platform
- A Daisy Pod board with a micro SD card
- A USB cable and serial terminal application (see note above)

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
git clone --recursive https://github.com/tone-3000/NAMPedal
cd NAMPedal
```

The `--recursive` flag pulls in the two submodules:

- **NeuralAmpModelerCore** — DSP engine (LSTM, WaveNet, ConvNet architectures)
- **nam-binary-loader** — `.namb` binary model parser

## Building

```bash
make
```

This cross-compiles for the STM32H750 (Cortex-M7) using `BOOT_QSPI` app type for the larger binary size NAM requires.

## Flashing

Put the Daisy Pod into DFU mode (hold BOOT, press RESET), then:

```bash
make program-dfu
```

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
NAMPedal: booting...
FS mount: OK
Loading model: nano_relu.namb
  file size: 1756 bytes
  read OK (1756 bytes)
  model ready (12 ms)
Benchmark: 142350 cycles for 48 frames (budget=480000)
  0.30 ms (deadline 1.00 ms)
Audio engine started
cb=48  cycles=142200  max=142350  gain=0.500  vol=0.750  BYPASS
```

### Running the Pedal

1. Copy a `.namb` model file to the root of a FAT32-formatted micro SD card, named `nano_relu.namb`
2. Insert the SD card into the Daisy Pod
3. Connect the Daisy Pod to your computer via USB and open a serial terminal (see above)
4. Power on or reset the board — the serial terminal must be open **before** this step
5. Wait for the `Audio engine started` message in the serial output
6. Connect your guitar to the audio input and an amp/headphones to the audio output

### Controls

| Control  | Function                                  |
|----------|-------------------------------------------|
| KNOB 1   | Gain (input level)                        |
| KNOB 2   | Volume (output level)                     |
| Button 1 | Toggle bypass (red LED = bypass, green LED = active) |

### LED Indicator

- **Red** — Effect is bypassed (clean passthrough)
- **Green** — Effect is active (NAM model processing)

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Board appears dead after flashing | No serial terminal connected; firmware is waiting for USB | Connect a serial terminal, then reset the board |
| `FS mount: FAILED` in serial output | SD card not inserted, not FAT32, or bad contact | Re-format the SD card as FAT32 and re-insert |
| `f_open failed` | Model file missing or wrong filename | Ensure the file is named `nano_relu.namb` at the root of the SD card |
| `file too large` | Model exceeds the 4 KB buffer | Use a smaller model (e.g. a `nano` or `feather` variant exported with NAM) |
| Audio glitches / dropouts | Model inference exceeds the 1 ms per-block deadline | Use a smaller/simpler model architecture; check the benchmark output |

## How it Works

- Audio is processed mono — the left input channel is fed through the NAM model and the output is duplicated to both stereo channels
- The model is loaded once at startup from the SD card into RAM, then `prewarm()` is called to stabilize initial state
- `NAM_SAMPLE` is defined as `float` (via `NAM_SAMPLE_FLOAT`) to match Daisy's audio buffers and halve memory usage compared to double precision
- The audio block size is 48 samples at 48 kHz, giving a 1 ms processing deadline per block
- FPU Flush-to-Zero and Default-NaN modes are enabled to avoid costly denormal handling on the Cortex-M7
