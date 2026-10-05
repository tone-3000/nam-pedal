// NAMPedal — Neural Amp Modeler running on a T3K Pedal (Daisy Seed,
// pure C inference)
//
// Reads A2-nano NAM models (.namb format) from a model bank in QSPI flash at
// MODEL_BANK_ADDR.  The bank is built and flashed with pack_models.py,
// independently of the firmware, so models can be swapped without recompiling.
//
// Signal chain: input -> noise gate -> input gain -> NAM -> loudness normalize -> 3-band EQ -> IR -> output volume.
//
// T3K Pedal controls:
//   INPUT_GAIN, OUTPUT_VOLUME, BASS, MID, TREBLE, NOISE_GATE_THRESHOLD knobs
//   FOOTSWITCH = bypass toggle
//   Noise gate is always active; its threshold is set entirely by the
//   NOISE_GATE_THRESHOLD knob (fully CCW = off).
//   ROTARY_1..4 = preset select (rotary switch, one position grounded at a time)
//   LED_STATUS = lit when active, off when bypassed, blinks while clipping
//   LED_PRESET_1..4 = lit for the currently selected preset

#include "daisy_seed.h"
#include "t3k_pedal.h"
#include "eq3band.h"
#include <cstring>

extern "C"
{
#include "nam_model.h"
}

using namespace daisy;
using namespace t3k_pedal;

#ifdef LOGGING
#define LOG(fmt, ...) hw.PrintLine(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...) \
    do                \
    {                 \
    } while(0)
#endif

// ── Model bank ────────────────────────────────────────────────────────────
//
// The bank lives at 0x90800000 — the first address past the firmware's
// declared QSPI region (0x90040000 + 7936 KB).  pack_models.py targets this.
//
// Layout (all little-endian):
//   Header (64 bytes):
//     u32 magic      0x504D414E "NAMP"
//     u32 version    1
//     u32 num_models
//     u32 reserved[13]
//   ModelEntry[num_models] (48 bytes each):
//     char name[32]  null-padded ASCII
//     u32  offset    from start of bank to this model's .namb data
//     u32  size      .namb byte count
//     u32  reserved[2]
//   .namb data blobs (4-byte aligned)

static constexpr uint32_t MODEL_BANK_ADDR  = 0x90600000UL;
static constexpr uint32_t MODEL_BANK_MAGIC = 0x504D414E; // "NAMP"

// Maximum IR length supported.  At 48 kHz: 256 taps ≈ 5.3 ms.
// Time-domain FIR with DTCM buffers costs n × L × 3.75 cycles per block.
// At 256 taps: 256 × 48 × 3.75 ≈ 46K cycles ≈ 10% of the 480K-cycle budget.
// Combined with NAM (~65%) and EQ (~0.5%) this leaves ~25% headroom.
// 512 taps pushed real-time cost to ~87% avg, causing callback overruns.
static constexpr uint32_t MAX_IR_TAPS = 256;

struct ModelBankHeader
{
    uint32_t magic;
    uint32_t version;
    uint32_t num_models;
    uint32_t reserved[13];
};
static_assert(sizeof(ModelBankHeader) == 64,
              "ModelBankHeader must be 64 bytes");

struct ModelEntry
{
    char     name[32];
    uint32_t offset;         // .namb data offset from bank start
    uint32_t size;           // .namb data size in bytes
    uint32_t ir_offset;      // IR float32 data offset (0 = no IR)
    uint32_t ir_num_samples; // IR length in float32 samples
};
static_assert(sizeof(ModelEntry) == 48, "ModelEntry must be 48 bytes");

// ── Hardware objects ──────────────────────────────────────────────────────

static DaisySeed hw;

// DaisySeed::GetPin() returns the legacy dsy_gpio_pin type (Pin converts
// implicitly to it, but not back); GPIO::Init() needs a daisy::Pin. GPIOPort
// and dsy_gpio_port share the same enumerator order, so this cast is safe.
static constexpr Pin ToPin(dsy_gpio_pin p)
{
    return Pin(static_cast<GPIOPort>(p.port), p.pin);
}

// Order must match T3kPedal::Knob (INPUT_GAIN, OUTPUT_VOLUME, BASS, MID,
// TREBLE, NOISE_GATE_THRESHOLD) — each entry must be an ADC-capable Seed
// pin (seed::A0..A11). The PCB wires the pots to ADC_0..ADC_5 in that order.
static constexpr Pin kKnobPins[6]
    = {seed::A0, seed::A1, seed::A2, seed::A3, seed::A4, seed::A5};
static Switch footswitch;
static Switch rotary1, rotary2, rotary3, rotary4;
static GPIO   led_status;
static GPIO   led_preset[4];

// Currently loaded preset (0-3), driven by the rotary switch position.
static volatile int current_preset = 0;

// Set from the audio callback when |sample| crosses the clip threshold;
// consumed by the main loop to drive the status LED's clip blink.
static constexpr float kClipThreshold = 0.98f;
static volatile bool   clip_detected  = false;

static NAM_DTCM Eq3Band eq;

// Set by the USB receive callback when a DFU-trigger byte ('D') is received.
// Checked in the main loop so ResetToBootloader() runs outside the ISR context.
static volatile bool dfu_requested = false;

static void OnUsbReceive(uint8_t* buf, uint32_t* len)
{
    for(uint32_t i = 0; i < *len; i++)
    {
        if(buf[i] == 'D')
        {
            dfu_requested = true;
            return;
        }
    }
}

static volatile float gain          = 1.0f;
static volatile float volume        = 1.0f;
static volatile bool  effect_active = true;

static nam_state_t   nam_state;
static volatile bool model_loaded = false;

static constexpr size_t kMaxBlockSize = NAM_MAX_BUFFER_SIZE;
static NAM_DTCM float   mono_in[kMaxBlockSize];
static NAM_DTCM float   mono_out[kMaxBlockSize];

// Time-domain FIR cabinet convolution.
//
// IR convolution buffers in DTCM (zero wait-state access).
// Coefficients are stored time-reversed for a cache-friendly forward scan.
// State buffer uses a double-write circular scheme (see IrProcess).
static NAM_DTCM float    g_ir_coeffs[MAX_IR_TAPS];
static NAM_DTCM float    g_ir_state[2 * MAX_IR_TAPS + NAM_MAX_BUFFER_SIZE - 1];
static volatile uint32_t g_ir_num_taps = 0;
static NAM_DTCM uint32_t g_ir_head = 0; // write pos, cycles [0, MAX_IR_TAPS)

// ── Noise gate ────────────────────────────────────────────────────────────
//
// Power-domain envelope follower with hold-time state machine, matching the
// plugin's Trigger/Gain design but without per-sample log10/pow:
//   - Threshold pre-converted to power in the control loop from the
//     NOISE_GATE_THRESHOLD knob; fully CCW sets threshold to 0 (off).
//   - Gate gain tracked in linear space; no dB conversion per sample.
//   - Operates on the raw input (before the gain knob) so the threshold is
//     gain-independent.
//   - Signal chain: input → [gate] → × gain → NAM → EQ → IR → output.

struct NoiseGate
{
    float    level      = 0.0f; // power envelope
    float    gain       = 0.0f; // current gate gain [0, 1]
    uint32_t held       = 0;    // samples spent holding open
    bool     holding    = false;
    float    alpha      = 0.0f; // envelope coeff
    float    beta       = 0.0f; // 1 - alpha
    float    open_rate  = 0.0f; // gain increase per sample (attack)
    float    close_rate = 0.0f; // gain decrease per sample (release)
    uint32_t hold_max   = 0;    // hold duration in samples
    float    threshold  = 0.0f; // power threshold, written by control loop

    void Init(float sample_rate)
    {
        alpha      = expf(-1.0f / (0.005f * sample_rate)); // 5 ms envelope
        beta       = 1.0f - alpha;
        open_rate  = 1.0f / (0.002f * sample_rate); // 2 ms attack
        close_rate = 1.0f / (0.100f * sample_rate); // 100 ms release
        hold_max   = static_cast<uint32_t>(0.050f * sample_rate); // 50 ms hold
        threshold  = 1e-6f; // -60 dBFS default
    }

    inline float Process(float x)
    {
        level = alpha * level + beta * (x * x);
        if(holding)
        {
            if(level < threshold)
            {
                if(++held >= hold_max)
                    holding = false;
            }
            else
            {
                held = 0;
            }
        }
        else
        {
            if(level >= threshold)
            {
                gain += open_rate;
                if(gain >= 1.0f)
                {
                    gain    = 1.0f;
                    holding = true;
                    held    = 0;
                }
            }
            else
            {
                gain -= close_rate;
                if(gain < 0.0f)
                    gain = 0.0f;
            }
        }
        return x * gain;
    }
};

static NAM_DTCM NoiseGate gate;

// A2-nano receptive field. Dilations × (kernel_size − 1) across all layers:
//   layers  0–13: kernel=6,  dilations [1,3,7,17,41,101,239,×2] → 4090
//   layers 14–15: kernel=15, dilations [1,13]                    →  196
//   layers 16–22: kernel=6,  dilations [1,3,7,17,41,101,239]     → 2045
static constexpr int kPrewarmSamples = 1 + 4090 + 196 + 2045;

static constexpr uint32_t NAMB_MAGIC = 0x4E414D42; // "NAMB"

// Loudness normalization, as in the TONE3000 plugin: bring the model's
// metadata.loudness to a fixed target, clamped to ±12 dB. Models without
// loudness metadata pass at unity.
static constexpr float kTargetLoudnessDb  = -18.0f;
static constexpr float kMaxNormalizeDb    = 12.0f;
static constexpr size_t kNambMetaSize     = 80;   // header 32 + metadata 48
static constexpr size_t kNambMetaFlagsOff = 35;
static constexpr size_t kNambLoudnessOff  = 44;   // f64
static constexpr uint8_t kMetaHasLoudness = 0x01;
static volatile float model_gain          = 1.0f; // post-NAM linear gain

static float NormalizeGain(const uint8_t* data, uint32_t size)
{
    if(size < kNambMetaSize || !(data[kNambMetaFlagsOff] & kMetaHasLoudness))
        return 1.0f;
    double loudness;
    memcpy(&loudness, data + kNambLoudnessOff, sizeof(loudness));
    if(!(loudness > -100.0 && loudness <= 0.0))
        return 1.0f;
    float db = kTargetLoudnessDb - (float)loudness;
    db       = fmaxf(-kMaxNormalizeDb, fminf(kMaxNormalizeDb, db));
    LOG("  loudness=%.1f dB -> normalize %+.1f dB", (float)loudness, db);
    return powf(10.0f, db / 20.0f);
}

// ── Model loading ─────────────────────────────────────────────────────────

static void Prewarm(int prewarm_samples)
{
    float zero_buf[NAM_MAX_BUFFER_SIZE];
    float out_buf[NAM_MAX_BUFFER_SIZE];
    memset(zero_buf, 0, sizeof(zero_buf));

    float* in_ptrs[NAM_IN_CHANNELS];
    float* out_ptrs[NAM_OUT_CHANNELS];
    for(int i = 0; i < NAM_IN_CHANNELS; i++)
        in_ptrs[i] = zero_buf;
    for(int i = 0; i < NAM_OUT_CHANNELS; i++)
        out_ptrs[i] = out_buf;

    int processed = 0;
    while(processed < prewarm_samples)
    {
        int n = NAM_MAX_BUFFER_SIZE;
        if(n > prewarm_samples - processed)
            n = prewarm_samples - processed;
        nam_process(&nam_state, (const float* const*)in_ptrs, out_ptrs, n);
        processed += n;
    }
}

// Load the model at the given preset index from the QSPI model bank.
// Returns true on success; on failure the audio callback stays in bypass.
static bool LoadModel(uint32_t idx)
{
    const ModelBankHeader* bank
        = reinterpret_cast<const ModelBankHeader*>(MODEL_BANK_ADDR);

    if(bank->magic != MODEL_BANK_MAGIC)
    {
        LOG("  no model bank at 0x%08lX (magic=0x%08lX)",
            (unsigned long)MODEL_BANK_ADDR,
            (unsigned long)bank->magic);
        return false;
    }
    if(bank->num_models == 0)
    {
        LOG("  model bank is empty");
        return false;
    }

    if(idx >= bank->num_models)
        idx = 0;

    const ModelEntry* entries = reinterpret_cast<const ModelEntry*>(
        reinterpret_cast<const uint8_t*>(bank) + sizeof(ModelBankHeader));
    const ModelEntry& e = entries[idx];

    // Model data in QSPI is 4-byte aligned: MODEL_BANK_ADDR is aligned,
    // header is 64 bytes (mult of 4), entries are 48 bytes (mult of 4),
    // and pack_models.py pads blobs to 4-byte boundaries.
    const uint8_t* data
        = reinterpret_cast<const uint8_t*>(MODEL_BANK_ADDR) + e.offset;
    uint32_t size = e.size;

    LOG("  model %lu/%lu: \"%s\" (%lu bytes)",
        (unsigned long)(idx + 1),
        (unsigned long)bank->num_models,
        e.name,
        (unsigned long)size);

    // The loader app always writes 4 entries (one per rotary position) and
    // marks unassigned presets with a zero-size blob. Treat those as "no
    // model": the caller leaves model_loaded=false and audio passes through.
    if(size == 0)
    {
        LOG("  preset is empty (no model assigned)");
        return false;
    }

    if(size < 32)
    {
        LOG("  data too small for NAMB header");
        return false;
    }

    uint32_t magic;
    memcpy(&magic, data, 4);
    if(magic != NAMB_MAGIC)
    {
        LOG("  bad NAMB magic: 0x%08lX", (unsigned long)magic);
        return false;
    }

    uint32_t weights_offset, num_weights;
    memcpy(&weights_offset, data + 12, 4);
    memcpy(&num_weights, data + 16, 4);

    LOG("  weights: offset=%lu count=%lu",
        (unsigned long)weights_offset,
        (unsigned long)num_weights);

    if(weights_offset + num_weights * 4u > size)
    {
        LOG("  weights extend beyond data");
        return false;
    }

    nam_init(&nam_state);

    const float* weights
        = reinterpret_cast<const float*>(data + weights_offset);
    int rc = nam_load_weights(weights, (int)num_weights);
    if(rc != 0)
    {
        LOG("  nam_load_weights failed (not an A2-nano model?)");
        return false;
    }

    model_gain = NormalizeGain(data, size);

    // ── Cabinet IR ────────────────────────────────────────────────────────
    // Clear state regardless of whether a new IR is present.
    g_ir_num_taps = 0;
    g_ir_head     = 0;
    memset(g_ir_state, 0, sizeof(g_ir_state));

    if(e.ir_offset != 0 && e.ir_num_samples > 0)
    {
        uint32_t taps = e.ir_num_samples;
        if(taps > MAX_IR_TAPS)
        {
            LOG("  IR: %lu taps truncated to %lu",
                (unsigned long)taps,
                (unsigned long)MAX_IR_TAPS);
            taps = MAX_IR_TAPS;
        }

        // IR float32 data sits directly in XIP QSPI (4-byte aligned by packer).
        const float* ir_src
            = reinterpret_cast<const float*>(MODEL_BANK_ADDR + e.ir_offset);

        // Store time-reversed so the inner loop scans forwards through both
        // the coefficient and state arrays (cache-friendly, compiler-vectorisable).
        for(uint32_t j = 0; j < taps; j++)
            g_ir_coeffs[j] = ir_src[taps - 1 - j];

        g_ir_num_taps = taps;
        LOG("  IR: %lu taps (%.1f ms)",
            (unsigned long)taps,
            (float)taps / 48.0f);
    }
    else
    {
        LOG("  IR: none");
    }

    LOG("  prewarming (%d samples)...", kPrewarmSamples);
    uint32_t t0 = System::GetNow();
    Prewarm(kPrewarmSamples);
    LOG("  model ready (%lu ms)", (unsigned long)(System::GetNow() - t0));
    return true;
}

// ── UI helpers ────────────────────────────────────────────────────────────

static void ToggleBypass()
{
    effect_active = !effect_active;
}

static void UpdatePresetLeds(int preset)
{
    for(int i = 0; i < 4; i++)
        led_preset[i].Write(i == preset);
}

static uint32_t GetBankNumModels()
{
    const ModelBankHeader* bank
        = reinterpret_cast<const ModelBankHeader*>(MODEL_BANK_ADDR);
    if(bank->magic != MODEL_BANK_MAGIC)
        return 0;
    return bank->num_models;
}

static void SwitchToModel(int idx)
{
    model_loaded = false; // audio falls through to bypass while we reload
    LOG("Switching to preset %d", idx + 1);
    model_loaded = LoadModel((uint32_t)idx);
    LOG("Model load: %s", model_loaded ? "OK" : "FAILED");
    current_preset = idx;
    UpdatePresetLeds(current_preset);
}

// Reads the 4 rotary switch position pins. Exactly one should read
// "pressed" (grounded) at a time; returns -1 if none or more than one
// read pressed (mid-rotation, or a wiring fault) so callers can ignore
// the ambiguous reading and keep the last known preset.
static int ReadRotaryPreset()
{
    bool pressed[4]
        = {rotary1.Pressed(), rotary2.Pressed(), rotary3.Pressed(), rotary4.Pressed()};
    int selected = -1;
    int count    = 0;
    for(int i = 0; i < 4; i++)
    {
        if(pressed[i])
        {
            selected = i;
            count++;
        }
    }
    return (count == 1) ? selected : -1;
}


// ── IR convolution ────────────────────────────────────────────────────────
//
// Double-write circular buffer FIR with time-reversed coefficients.
// All data (coefficients and state) lives in DTCM for zero wait-state access.
//
// Each new sample is written at position p and p + MAX_IR_TAPS so that any
// n-sample read window is always contiguous — no wrap-around logic in the
// inner loop, no memmove on block exit.  src and dst must be distinct.

static void IrProcess(const float* __restrict__ src,
                      float* __restrict__ dst,
                      uint32_t block_size)
{
    const uint32_t n    = g_ir_num_taps;
    const uint32_t head = g_ir_head;

    memcpy(g_ir_state + head, src, block_size * sizeof(float));
    memcpy(g_ir_state + head + MAX_IR_TAPS, src, block_size * sizeof(float));

    for(uint32_t i = 0; i < block_size; i++)
    {
        const float* s      = g_ir_state + (head + i + MAX_IR_TAPS - (n - 1));
        const float* coeffs = g_ir_coeffs;
        // Four independent accumulators hide the M7 FPU's ~5-cycle MAC latency.
        float acc0 = 0.0f, acc1 = 0.0f, acc2 = 0.0f, acc3 = 0.0f;

        uint32_t k = n >> 2;
        while(k--)
        {
            acc0 += coeffs[0] * s[0];
            acc1 += coeffs[1] * s[1];
            acc2 += coeffs[2] * s[2];
            acc3 += coeffs[3] * s[3];
            coeffs += 4;
            s += 4;
        }
        float acc = (acc0 + acc1) + (acc2 + acc3);
        k         = n & 3;
        while(k--)
            acc += *coeffs++ * *s++;

        dst[i] = acc;
    }

    g_ir_head = head + block_size;
    if(g_ir_head >= MAX_IR_TAPS)
        g_ir_head -= MAX_IR_TAPS;
}

// ── Audio callback ────────────────────────────────────────────────────────

static volatile uint32_t cb_count          = 0;
static volatile uint32_t cb_process_cycles = 0;
static volatile uint32_t cb_max_cycles     = 0;

static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    cb_count++;
    size_t num_frames = size / 2;

    if(model_loaded && effect_active)
    {
        for(size_t i = 0; i < num_frames; i++)
        {
            float x = in[i * 2];
            if(fabsf(x) >= kClipThreshold)
                clip_detected = true;
            mono_in[i] = gate.Process(x) * gain;
        }

        float* input_ptr  = mono_in;
        float* output_ptr = mono_out;

        uint32_t cyc0 = DWT->CYCCNT;

        // Signal chain matches the NAM plugin: NAM → normalize → EQ → IR.
        nam_process(&nam_state,
                    (const float* const*)&input_ptr,
                    &output_ptr,
                    num_frames);

        // Loudness normalization and EQ in-place on mono_out (NAM output).
        const float norm = model_gain;
        for(size_t i = 0; i < num_frames; i++)
            mono_out[i] = eq.Process(mono_out[i] * norm);

        // Cabinet IR (if loaded): mono_out → mono_in.
        // mono_in is free — NAM has consumed it — so use it as scratch.
        const float* ir_out = mono_out;
        if(g_ir_num_taps > 0)
        {
            IrProcess(mono_out, mono_in, num_frames);
            ir_out = mono_in;
        }

        uint32_t cyc1 = DWT->CYCCNT;

        uint32_t elapsed  = cyc1 - cyc0;
        cb_process_cycles = elapsed;
        if(elapsed > cb_max_cycles)
            cb_max_cycles = elapsed;

        for(size_t i = 0; i < num_frames; i++)
        {
            float s = ir_out[i] * volume;
            if(fabsf(s) >= kClipThreshold)
                clip_detected = true;
            out[i * 2]     = s;
            out[i * 2 + 1] = s;
        }
    }
    else
    {
        for(size_t i = 0; i < num_frames; i++)
        {
            float x = in[i * 2];
            if(fabsf(x) >= kClipThreshold)
                clip_detected = true;
            float s = x * volume;
            if(fabsf(s) >= kClipThreshold)
                clip_detected = true;
            out[i * 2]     = s;
            out[i * 2 + 1] = s;
        }
    }
}

// ── Entry point ───────────────────────────────────────────────────────────

int main(void)
{
    hw.Init();

    // FPDSCR sets the default FPSCR for all new FPU contexts (including ISRs).
    uint32_t fpscr = __get_FPSCR();
    fpscr |= (1U << 24) | (1U << 25);
    __set_FPSCR(fpscr);
    volatile uint32_t* FPDSCR
        = reinterpret_cast<volatile uint32_t*>(0xE000EF3C);
    *FPDSCR |= (1U << 24) | (1U << 25);

    // Always init USB CDC so the DFU serial command works regardless of LOGGING.
    hw.StartLog(false);
    hw.usb_handle.SetReceiveCallback(OnUsbReceive, UsbHandle::FS_INTERNAL);
#ifdef LOGGING
    System::Delay(2000);
    LOG("NAMPedal (A2-nano C): booting...");
#endif

    hw.SetAudioBlockSize(kMaxBlockSize);
    eq.Init(hw.AudioSampleRate());
    gate.Init(hw.AudioSampleRate());

    AdcChannelConfig adc_cfg[6];
    for(size_t i = 0; i < 6; i++)
        adc_cfg[i].InitSingle(kKnobPins[i]);
    hw.adc.Init(adc_cfg, 6);

    footswitch.Init(hw.GetPin(T3kPedal::FOOTSWITCH));
    rotary1.Init(hw.GetPin(T3kPedal::ROTARY_1),
                 0.f,
                 Switch::TYPE_TOGGLE,
                 Switch::POLARITY_INVERTED,
                 Switch::PULL_UP);
    rotary2.Init(hw.GetPin(T3kPedal::ROTARY_2),
                 0.f,
                 Switch::TYPE_TOGGLE,
                 Switch::POLARITY_INVERTED,
                 Switch::PULL_UP);
    rotary3.Init(hw.GetPin(T3kPedal::ROTARY_3),
                 0.f,
                 Switch::TYPE_TOGGLE,
                 Switch::POLARITY_INVERTED,
                 Switch::PULL_UP);
    rotary4.Init(hw.GetPin(T3kPedal::ROTARY_4),
                 0.f,
                 Switch::TYPE_TOGGLE,
                 Switch::POLARITY_INVERTED,
                 Switch::PULL_UP);
    led_status.Init(ToPin(hw.GetPin(T3kPedal::LED_STATUS)), GPIO::Mode::OUTPUT);
    led_preset[0].Init(ToPin(hw.GetPin(T3kPedal::LED_PRESET_1)), GPIO::Mode::OUTPUT);
    led_preset[1].Init(ToPin(hw.GetPin(T3kPedal::LED_PRESET_2)), GPIO::Mode::OUTPUT);
    led_preset[2].Init(ToPin(hw.GetPin(T3kPedal::LED_PRESET_3)), GPIO::Mode::OUTPUT);
    led_preset[3].Init(ToPin(hw.GetPin(T3kPedal::LED_PRESET_4)), GPIO::Mode::OUTPUT);

    // Debounce the rotary switch for a few cycles so its initial reading
    // (used to pick the boot preset) isn't taken from uninitialized state.
    for(int i = 0; i < 10; i++)
    {
        rotary1.Debounce();
        rotary2.Debounce();
        rotary3.Debounce();
        rotary4.Debounce();
        System::Delay(1);
    }
    int boot_preset = ReadRotaryPreset();
    if(boot_preset < 0)
        boot_preset = 0;
    current_preset = boot_preset;

    LOG("Loading model from bank at 0x%08lX", (unsigned long)MODEL_BANK_ADDR);
    model_loaded = LoadModel((uint32_t)current_preset);
    LOG("Model load: %s", model_loaded ? "OK" : "FAILED");
    UpdatePresetLeds(current_preset);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if(model_loaded)
    {
        constexpr int      kRuns   = 100;
        constexpr uint32_t kBudget = 480000;

        uint32_t nam_sum = 0, nam_max = 0;
        uint32_t eq_sum = 0, eq_max = 0;
        uint32_t ir_sum = 0, ir_max = 0;
        uint32_t tot_max = 0;

        for(int run = 0; run < kRuns; run++)
        {
            float* input_ptr  = mono_in;
            float* output_ptr = mono_out;
            memset(mono_in, 0, sizeof(mono_in));

            DWT->CYCCNT = 0;
            nam_process(&nam_state,
                        (const float* const*)&input_ptr,
                        &output_ptr,
                        kMaxBlockSize);
            uint32_t cy_nam = DWT->CYCCNT;

            DWT->CYCCNT = 0;
            for(size_t i = 0; i < kMaxBlockSize; i++)
                mono_out[i] = eq.Process(mono_out[i]);
            uint32_t cy_eq = DWT->CYCCNT;

            uint32_t cy_ir = 0;
            if(g_ir_num_taps > 0)
            {
                DWT->CYCCNT = 0;
                IrProcess(mono_out, mono_in, kMaxBlockSize);
                cy_ir = DWT->CYCCNT;
            }

            nam_sum += cy_nam;
            if(cy_nam > nam_max)
                nam_max = cy_nam;
            eq_sum += cy_eq;
            if(cy_eq > eq_max)
                eq_max = cy_eq;
            ir_sum += cy_ir;
            if(cy_ir > ir_max)
                ir_max = cy_ir;

            uint32_t tot = cy_nam + cy_eq + cy_ir;
            if(tot > tot_max)
                tot_max = tot;
        }

        uint32_t nam_avg = nam_sum / kRuns;
        uint32_t eq_avg  = eq_sum / kRuns;
        uint32_t ir_avg  = ir_sum / kRuns;
        uint32_t tot_avg = nam_avg + eq_avg + ir_avg;

        LOG("Benchmark (%d blocks x %u frames, budget=%lu cy):",
            kRuns,
            (unsigned)kMaxBlockSize,
            (unsigned long)kBudget);
        LOG("           avg cy   max cy   avg%%   max%%");
        LOG("  NAM  : %7lu  %7lu  %4.1f%%  %4.1f%%",
            (unsigned long)nam_avg,
            (unsigned long)nam_max,
            100.0f * (float)nam_avg / kBudget,
            100.0f * (float)nam_max / kBudget);
        LOG("  EQ   : %7lu  %7lu  %4.1f%%  %4.1f%%",
            (unsigned long)eq_avg,
            (unsigned long)eq_max,
            100.0f * (float)eq_avg / kBudget,
            100.0f * (float)eq_max / kBudget);
        if(g_ir_num_taps > 0)
            LOG("  IR   : %7lu  %7lu  %4.1f%%  %4.1f%%  (%lu taps)",
                (unsigned long)ir_avg,
                (unsigned long)ir_max,
                100.0f * (float)ir_avg / kBudget,
                100.0f * (float)ir_max / kBudget,
                (unsigned long)g_ir_num_taps);
        else
            LOG("  IR   :      --       --     --     --  (none)");
        LOG("  TOTAL: %7lu  %7lu  %4.1f%%  %4.1f%%  (%.3f ms avg)",
            (unsigned long)tot_avg,
            (unsigned long)tot_max,
            100.0f * (float)tot_avg / kBudget,
            100.0f * (float)tot_max / kBudget,
            (float)tot_avg / 480000.0f);
    }

    hw.adc.Start();
    hw.StartAudio(AudioCallback);
    LOG("Audio engine started");

    led_status.Write(true); // start active

    float last_bass = 999.0f, last_mid = 999.0f, last_treble = 999.0f;
    float last_ng = -1.0f; // tracks NOISE_GATE_THRESHOLD knob while gate is on
#ifdef LOGGING
    uint32_t last_print = System::GetNow();
#endif

    // Status LED clip-blink state.
    static constexpr uint32_t kClipHoldMs       = 300;
    static constexpr uint32_t kClipBlinkHalfMs  = 100;
    uint32_t clip_hold_until = 0;
    uint32_t next_blink_toggle = 0;
    bool     status_blink_on = false;

    for(;;)
    {
        if(dfu_requested)
        {
            for(int i = 0; i < 2; i++)
            {
                led_status.Write(true);
                for(int p = 0; p < 4; p++)
                    led_preset[p].Write(true);
                System::Delay(150);
                led_status.Write(false);
                for(int p = 0; p < 4; p++)
                    led_preset[p].Write(false);
                System::Delay(150);
            }
            System::ResetToBootloader(
                System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
        }

        footswitch.Debounce();
        rotary1.Debounce();
        rotary2.Debounce();
        rotary3.Debounce();
        rotary4.Debounce();

        volume = hw.adc.GetFloat(T3kPedal::OUTPUT_VOLUME);

        float knob_gain = hw.adc.GetFloat(T3kPedal::INPUT_GAIN);
        gain             = knob_gain * 2.0f;

        // Noise gate threshold tracks the knob directly (CCW = -80 dBFS,
        // CW = -30 dBFS), with the bottom of the knob's travel shutting
        // the gate off entirely (threshold 0 = level always above it).
        static constexpr float kGateOffZone = 0.02f;
        float knob_gate_threshold = hw.adc.GetFloat(T3kPedal::NOISE_GATE_THRESHOLD);
        if(knob_gate_threshold != last_ng)
        {
            gate.threshold = (knob_gate_threshold <= kGateOffZone)
                                  ? 0.0f
                                  : powf(10.0f,
                                         (-80.0f + knob_gate_threshold * 50.0f) / 10.0f);
            last_ng = knob_gate_threshold;
        }

        // Ranges match the NAM plugin ToneStack: bass ±20 dB, mid ±15 dB, treble ±10 dB.
        float bass   = (hw.adc.GetFloat(T3kPedal::BASS) - 0.5f) * 40.0f;
        float mid    = (hw.adc.GetFloat(T3kPedal::MID) - 0.5f) * 30.0f;
        float treble = (hw.adc.GetFloat(T3kPedal::TREBLE) - 0.5f) * 20.0f;
        if(bass != last_bass)
        {
            eq.SetBass(bass);
            last_bass = bass;
        }
        if(mid != last_mid)
        {
            eq.SetMid(mid);
            last_mid = mid;
        }
        if(treble != last_treble)
        {
            eq.SetTreble(treble);
            last_treble = treble;
        }

        if(footswitch.FallingEdge())
            ToggleBypass();

        // Rotary preset select: switch only on an unambiguous single-position
        // reading that differs from the current preset, and only if that
        // position actually has a model in the bank.
        int rotary_preset = ReadRotaryPreset();
        if(rotary_preset >= 0 && rotary_preset != current_preset
           && (uint32_t)rotary_preset < GetBankNumModels())
            SwitchToModel(rotary_preset);

        uint32_t now = System::GetNow();

        // Status LED: blink while clipping, otherwise reflect bypass state.
        if(clip_detected)
        {
            clip_detected    = false;
            clip_hold_until  = now + kClipHoldMs;
        }
        if((int32_t)(clip_hold_until - now) > 0)
        {
            if(now - next_blink_toggle >= kClipBlinkHalfMs)
            {
                status_blink_on   = !status_blink_on;
                next_blink_toggle = now;
                led_status.Write(status_blink_on);
            }
        }
        else
        {
            led_status.Write(effect_active);
        }

#ifdef LOGGING
        if(now - last_print >= 1000)
        {
            // knob_gain/knob_gate_threshold already read above; re-read
            // the others for the log snapshot
            float knob_volume = hw.adc.GetFloat(T3kPedal::OUTPUT_VOLUME);
            float knob_bass   = hw.adc.GetFloat(T3kPedal::BASS);
            float knob_mid    = hw.adc.GetFloat(T3kPedal::MID);
            float knob_treble = hw.adc.GetFloat(T3kPedal::TREBLE);
            LOG("cb=%lu  cycles=%lu  max=%lu  %s  preset=%d",
                (unsigned long)cb_count,
                (unsigned long)cb_process_cycles,
                (unsigned long)cb_max_cycles,
                effect_active ? "ACTIVE" : "BYPASS",
                current_preset + 1);
            LOG("  knobs gain=%.3f vol=%.3f bass=%.3f mid=%.3f treble=%.3f gate=%.3f",
                knob_gain,
                knob_volume,
                knob_bass,
                knob_mid,
                knob_treble,
                knob_gate_threshold);
            LOG("  ftsw=%d  rot1=%d rot2=%d rot3=%d rot4=%d",
                footswitch.Pressed() ? 1 : 0,
                rotary1.Pressed() ? 1 : 0,
                rotary2.Pressed() ? 1 : 0,
                rotary3.Pressed() ? 1 : 0,
                rotary4.Pressed() ? 1 : 0);
            // GPIO::Read() returns the pin's *input data register*, i.e. the
            // actual voltage on the pin even in output mode. A pin commanded
            // high that reads back 0 is being pulled down externally.
            LOG("  led pins (readback): status=%d p1=%d p2=%d p3=%d p4=%d",
                led_status.Read() ? 1 : 0,
                led_preset[0].Read() ? 1 : 0,
                led_preset[1].Read() ? 1 : 0,
                led_preset[2].Read() ? 1 : 0,
                led_preset[3].Read() ? 1 : 0);
            LOG("  gain=%.2f vol=%.2f eq[%.1f %.1f %.1f]",
                gain,
                volume,
                bass,
                mid,
                treble);
            last_print = now;
        }
#endif
    }
}
