// NAMPedal — Neural Amp Modeler running on a Daisy Pod
//
// This example loads a NAM model in .namb (binary) format from the SD card
// and applies it to incoming audio in real time. Knob 1 controls input gain
// and Knob 2 controls output volume. SW1 toggles bypass.
//
// To adapt this to your own hardware (e.g. Daisy Seed with a custom PCB):
//   1. Replace DaisyPod with your board support class (DaisySeed, DaisyPetal, etc.)
//   2. Adjust the audio block size and sample rate for your use case.
//   3. Map your own controls (pots, switches, MIDI) in the main loop.

#include "daisy_pod.h"
#include "NAM/dsp.h"            // nam::DSP — the core inference engine
#include "NAM/activations.h"    // nam::activations::Activation::enable_fast_tanh
#include "namb/get_dsp_namb.h"  // nam::get_dsp_namb — deserialises a .namb blob
#include <memory>

using namespace daisy;

// Hardware handle — swap this type for a different Daisy board.
static DaisyPod hw;

// The loaded NAM model. nullptr until a model is successfully loaded.
static std::unique_ptr<nam::DSP> model;

// Gain (input drive) and volume (output level), both in [0, 1].
static float gain = 1.0f;
static float volume = 1.0f;

// Bypass flag — toggled by SW1. true = model active, false = clean passthrough.
static volatile bool effect_active = false;

// Scratch buffers for mono audio passed to the model.
static constexpr size_t kMaxBlockSize = 48;
static NAM_SAMPLE mono_in[kMaxBlockSize];
static NAM_SAMPLE mono_out[kMaxBlockSize];

// SD card and FAT filesystem handles for loading .namb files.
static SdmmcHandler sdcard;
static FatFSInterface fsi;

// FIL and read buffer as plain globals — default .bss is in AXI SRAM,
// which is DMA-accessible (DTCMRAM stack variables are not).
static FIL sd_file;
static constexpr size_t kMaxFileSize = 4 * 1024; // 4 KB max model size
static uint8_t file_read_buf[kMaxFileSize];

// Load a .namb model from the SD card into RAM and prepare it for inference.
// Returns true on success. On failure the global `model` remains unchanged.
static bool LoadModel(const char* path)
{
    FRESULT fr;

    fr = f_open(&sd_file, path, FA_READ);
    if (fr != FR_OK)
    {
        hw.seed.PrintLine("  f_open failed: %d", (int)fr);
        return false;
    }

    UINT file_size = f_size(&sd_file);
    hw.seed.PrintLine("  file size: %u bytes", (unsigned)file_size);
    if (file_size > kMaxFileSize)
    {
        hw.seed.PrintLine("  file too large (max %u)", (unsigned)kMaxFileSize);
        f_close(&sd_file);
        return false;
    }

    // Read in 512-byte chunks to avoid SDMMC DMA transfer size issues.
    UINT total_read = 0;
    while (total_read < file_size)
    {
        UINT chunk = file_size - total_read;
        if (chunk > 512)
            chunk = 512;

        UINT bytes_read;
        fr = f_read(&sd_file, file_read_buf + total_read, chunk, &bytes_read);
        if (fr != FR_OK)
        {
            hw.seed.PrintLine("  f_read failed at offset %u: fr=%d",
                              (unsigned)total_read, (int)fr);
            f_close(&sd_file);
            return false;
        }
        total_read += bytes_read;
    }
    f_close(&sd_file);
    hw.seed.PrintLine("  read OK (%u bytes)", (unsigned)total_read);

    // Deserialise the .namb blob into a ready-to-run DSP object.
    uint32_t t0 = System::GetNow();
    try
    {
        model = nam::get_dsp_namb(file_read_buf, total_read);
    }
    catch (const std::exception& e)
    {
        hw.seed.PrintLine("  EXCEPTION: %s", e.what());
        return false;
    }

    if (model)
    {
        model->ResetAndPrewarm(48000.0, 48);
        hw.seed.PrintLine("  model ready (%lu ms)",
                          (unsigned long)(System::GetNow() - t0));
    }
    else
    {
        hw.seed.PrintLine("  get_dsp_namb returned null");
    }

    return model != nullptr;
}

// Callback diagnostics.
static volatile uint32_t cb_count = 0;
static volatile uint32_t cb_process_cycles = 0;
static volatile uint32_t cb_max_cycles = 0;

// Audio callback — called by the Daisy audio engine once per block.
// `in` and `out` are interleaved stereo buffers (L, R, L, R, …).
// `size` is the total number of *samples* (frames × 2 channels).
static void AudioCallback(AudioHandle::InterleavingInputBuffer in,
                           AudioHandle::InterleavingOutputBuffer out,
                           size_t size)
{
    // Force FZ/DN in this ISR context.
    __set_FPSCR(__get_FPSCR() | (1U << 24) | (1U << 25));

    cb_count++;
    size_t num_frames = size / 2;

    if (model && effect_active)
    {
        // De-interleave: take the left channel and apply input gain.
        for (size_t i = 0; i < num_frames; i++)
            mono_in[i] = in[i * 2] * gain;

        NAM_SAMPLE* input_ptr = mono_in;
        NAM_SAMPLE* output_ptr = mono_out;

        uint32_t cyc0 = DWT->CYCCNT;
        model->process(&input_ptr, &output_ptr, num_frames);
        uint32_t cyc1 = DWT->CYCCNT;

        uint32_t elapsed = cyc1 - cyc0;
        cb_process_cycles = elapsed;
        if (elapsed > cb_max_cycles)
            cb_max_cycles = elapsed;

        // Re-interleave: copy the mono result to both L and R outputs.
        for (size_t i = 0; i < num_frames; i++)
        {
            out[i * 2] = mono_out[i] * volume;
            out[i * 2 + 1] = mono_out[i] * volume;
        }
    }
    else
    {
        // Bypass / no model — clean passthrough (left channel to both outputs).
        for (size_t i = 0; i < num_frames; i++)
        {
            float s = in[i * 2] * volume;
            out[i * 2] = s;
            out[i * 2 + 1] = s;
        }
    }
}

int main(void)
{
    // Initialise the hardware (clocks, peripherals, codec).
    hw.Init();

    // Enable Flush-to-Zero (FZ) and Default-NaN (DN) on the FPU.
    // FPDSCR sets the default FPSCR for all new FPU contexts (including ISRs).
    uint32_t fpscr = __get_FPSCR();
    fpscr |= (1U << 24) | (1U << 25);  // FZ | DN
    __set_FPSCR(fpscr);
    volatile uint32_t* FPDSCR = reinterpret_cast<volatile uint32_t*>(0xE000EF3C);
    *FPDSCR |= (1U << 24) | (1U << 25);

    // Start USB serial logging and wait briefly for the connection.
    hw.seed.StartLog(true);
    System::Delay(2000);
    hw.seed.PrintLine("NAMPedal: booting...");

    hw.SetAudioBlockSize(48);

    // --- SD card setup ---------------------------------------------------
    SdmmcHandler::Config sd_cfg;
    sd_cfg.Defaults();
    sdcard.Init(sd_cfg);

    fsi.Init(FatFSInterface::Config::MEDIA_SD);
    FATFS& fs = fsi.GetSDFileSystem();
    FRESULT mount_res = f_mount(&fs, fsi.GetSDPath(), 1);
    hw.seed.PrintLine("FS mount: %s", mount_res == FR_OK ? "OK" : "FAILED");

    // Enable fast tanh approximation for models that use it.
    nam::activations::Activation::enable_fast_tanh();

    // Load a .namb model from the root of the SD card.
    hw.seed.PrintLine("Loading model: nano_relu.namb");
    bool loaded = LoadModel("nano_relu.namb");
    hw.seed.PrintLine("Model load: %s", loaded ? "OK" : "FAILED");

    // Enable the DWT cycle counter for profiling.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Benchmark: run process() once before audio starts.
    if (loaded)
    {
        for (size_t i = 0; i < kMaxBlockSize; i++)
            mono_in[i] = 0.0f;

        NAM_SAMPLE* input_ptr = mono_in;
        NAM_SAMPLE* output_ptr = mono_out;

        DWT->CYCCNT = 0;
        model->process(&input_ptr, &output_ptr, kMaxBlockSize);
        uint32_t cycles = DWT->CYCCNT;

        hw.seed.PrintLine("Benchmark: %lu cycles for %u frames (budget=480000)",
                          (unsigned long)cycles, (unsigned)kMaxBlockSize);
        hw.seed.PrintLine("  %.2f ms (deadline 1.00 ms)",
                          (float)cycles / 480000.0f);
    }

    // Start the ADC (needed for knob/CV readings) and the audio engine.
    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    hw.seed.PrintLine("Audio engine started");

    // Set initial LED state: red = bypassed.
    hw.led1.Set(1.0f, 0.0f, 0.0f);
    hw.UpdateLeds();

    // Main loop — runs forever, reading controls each iteration.
    uint32_t last_print = System::GetNow();
    for (;;)
    {
        hw.ProcessAllControls();
        gain = hw.GetKnobValue(DaisyPod::KNOB_1);
        volume = hw.GetKnobValue(DaisyPod::KNOB_2);

        // SW1 toggles bypass.
        if (hw.button1.RisingEdge())
        {
            effect_active = !effect_active;
            if (effect_active)
                hw.led1.Set(0.0f, 1.0f, 0.0f);
            else
                hw.led1.Set(1.0f, 0.0f, 0.0f);
            hw.UpdateLeds();
        }

        // Print debug info once per second.
        uint32_t now = System::GetNow();
        if (now - last_print >= 1000)
        {
            hw.seed.PrintLine("cb=%lu  cycles=%lu  max=%lu  gain=%.3f  vol=%.3f  %s",
                (unsigned long)cb_count,
                (unsigned long)cb_process_cycles,
                (unsigned long)cb_max_cycles,
                gain, volume,
                effect_active ? "ACTIVE" : "BYPASS");
            last_print = now;
        }
    }
}
