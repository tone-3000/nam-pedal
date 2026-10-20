// t3k_pedal — Neural Amp Modeler firmware for the TONE3000 × Daisy DIY Pedal
// (T3K Pedal: a Daisy Seed on the nam-pedal-diy board).
//
// Inference uses the A2-Lite Cortex-M7 engine from tone-3000/embedded (git
// submodule at embedded/): dependency-free C with generated Thumb-2/FPv5
// kernels.  Engine code runs from ITCM, captures and engine instances live in
// DTCM, and the ring buffers in AXI SRAM, as the engine's README recommends.
//
// Presets are read from a bank in QSPI flash at MODEL_BANK_ADDR, written by
// the companion app (app/) or pack_models.py independently of the firmware.
// Each preset is one or two blocks:
//   block 1: an A2-Lite NAM capture (.namb)
//   block 2: another A2-Lite capture (e.g. pedal -> amp), or a cabinet IR
//
// Signal chain (gain staging mirrors the TONE3000 plugin, with the pedal's
// known analog levels standing in for the plugin's calibration setting):
//
//   input -> noise gate -> INPUT knob
//     -> [input calibration]  NAM 1  [calibrated hand-off | loudness normalize]
//     -> ( [input calibration]  NAM 2  [loudness normalize]
//        | cab IR, normalized to unity in-band gain )
//     -> 3-band EQ -> OUTPUT knob
//
//   input calibration   gain = kInputLevelDbu - capture input_level_dbu, so
//                       a capture with calibration data is hit at the level
//                       its creator used. Captures without it pass at unity.
//   calibrated hand-off NAM 1 into NAM 2 only, when NAM 1 carries
//                       output_level_dbu: gain = output_level_dbu -
//                       kInputLevelDbu, so NAM 2's calibration lands on the
//                       true level of plugging device A into device B.
//   loudness normalize  the last NAM block always: capture loudness ->
//                       -18 dB target, clamped to ±12 dB (unity without data).
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
#include <cmath>
#include <cstring>

extern "C"
{
#include "nam_a2lite.h"
}
#include "namb_reader.h"

#include "daisyconvol/daisyconvol.h"
#include "daisyconvol_daisy/seed.h"

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

// ── Preset bank ───────────────────────────────────────────────────────────
//
// The bank lives at 0x90600000, past the firmware's QSPI region.  The
// companion app and pack_models.py target this address.
//
// Layout (all little-endian):
//   Header (64 bytes):
//     u32 magic        0x504D414E "NAMP"
//     u32 version      2
//     u32 num_presets
//     u32 reserved[13]
//   PresetEntry[num_presets] (64 bytes each):
//     char name[32]        null-padded ASCII
//     u32  nam_offset      block 1: offset from bank start to the .namb
//     u32  nam_size        block 1: .namb byte count (0 = empty preset)
//     u32  nam2_offset     block 2 as NAM: .namb offset (0 = none)
//     u32  nam2_size       block 2 as NAM: .namb byte count
//     u32  ir_offset       block 2 as IR: float32 sample offset (0 = none)
//     u32  ir_num_samples  block 2 as IR: sample count
//     u32  nam_model_id    TONE3000 model id of block 1 (0 = unknown)
//     u32  block2_model_id TONE3000 model id of block 2, NAM or IR (0 = unknown)
//   data blobs (4-byte aligned)
//
// The model ids are bookkeeping for the companion app (the 'B' serial command
// below reports them so the app can rebuild its preset state from a plugged-in
// pedal); the firmware never acts on them.

static constexpr uint32_t MODEL_BANK_ADDR    = 0x90600000UL;
static constexpr uint32_t MODEL_BANK_MAGIC   = 0x504D414E; // "NAMP"
static constexpr uint32_t MODEL_BANK_VERSION = 2;

// Maximum IR length supported.  At 48 kHz: 4096 samples ≈ 85 ms.  Must match
// DAISYCONVOL_MAX_IR in the Makefile and MAX_IR_TAPS in pack_models.py and the
// app; longer IRs in the bank are truncated at load.
static constexpr uint32_t kMaxIrLen = 4096;

struct ModelBankHeader
{
    uint32_t magic;
    uint32_t version;
    uint32_t num_presets;
    uint32_t reserved[13];
};
static_assert(sizeof(ModelBankHeader) == 64,
              "ModelBankHeader must be 64 bytes");

struct PresetEntry
{
    char     name[32];
    uint32_t nam_offset;     // block 1 .namb offset from bank start
    uint32_t nam_size;       // block 1 .namb size in bytes (0 = empty preset)
    uint32_t nam2_offset;    // block 2 .namb offset (0 = no second NAM)
    uint32_t nam2_size;      // block 2 .namb size in bytes
    uint32_t ir_offset;       // block 2 IR float32 data offset (0 = no IR)
    uint32_t ir_num_samples;  // block 2 IR length in float32 samples
    uint32_t nam_model_id;    // TONE3000 model id, block 1 (0 = unknown)
    uint32_t block2_model_id; // TONE3000 model id, block 2 (0 = unknown)
};
static_assert(sizeof(PresetEntry) == 64, "PresetEntry must be 64 bytes");

// ── Hardware objects ──────────────────────────────────────────────────────

static DaisySeed hw;

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

// USB serial commands, one byte each. Flags are set by the receive callback
// and acted on in the main loop, outside the ISR context.
//   'D'  reset into the Daisy bootloader (DFU) — used by the app and
//        `make program-dfu PORT=` / `pack_models.py --port` before flashing
//   'B'  print the preset bank directory (see PrintBankDirectory) — used by
//        the app's "Sync from pedal"
static volatile bool dfu_requested  = false;
static volatile bool bank_requested = false;

static void OnUsbReceive(uint8_t* buf, uint32_t* len)
{
    for(uint32_t i = 0; i < *len; i++)
    {
        if(buf[i] == 'D')
        {
            dfu_requested = true;
            return;
        }
        if(buf[i] == 'B')
            bank_requested = true;
    }
}

static volatile float gain          = 1.0f;
static volatile float volume        = 1.0f;
static volatile bool  effect_active = true;

// A2-Lite engine memory for the two NAM blocks, placed as the engine's README
// recommends: captures and instances (head buffer, scratch, layer
// descriptors) in DTCM; the ~100 KB of ring buffers per instance in AXI SRAM
// behind the D-cache.  Block 2's instance is only used when a preset chains
// two captures.
static constexpr int                 kNumNamBlocks = 2;
static NAM_DTCM nam_a2lite_weights_t nam_weights[kNumNamBlocks];
static NAM_DTCM nam_a2lite_t         nam_inst[kNumNamBlocks];
static nam_a2lite_state_t            nam_state[kNumNamBlocks];
// Per-block gain staging, computed at preset load (see StageChain):
//   nam_pre_gain   into the model: input calibration
//   nam_post_gain  out of the model: calibrated hand-off or loudness normalize
static volatile float                nam_pre_gain[kNumNamBlocks]  = {1.0f, 1.0f};
static volatile float                nam_post_gain[kNumNamBlocks] = {1.0f, 1.0f};
static volatile bool                 nam2_loaded  = false; // block 2 is a NAM capture
static volatile bool                 model_loaded = false;

static constexpr size_t kMaxBlockSize = NAM_A2LITE_MAX_BUFFER_SIZE;
static NAM_DTCM float   mono_in[kMaxBlockSize];
static NAM_DTCM float   mono_out[kMaxBlockSize];

// Cabinet IR convolution: daisyConvol's zero-latency partitioned convolver.
//
// The engine needs a power-of-two block, so it runs 16-sample blocks, three
// per 48-frame callback (see IrProcess); its default geometry for a 4096-sample
// IR is then 16 x 16 + 30 x 128 partitions, all inside the callback. Arenas
// as its README recommends: the per-callback state (fast) and the FFT tables
// in DTCM, the stage-1 spectra and delay line (medium, 60 KB) in AXI SRAM.
using daisyconvol::ConvEngine;
static constexpr size_t kConvBlock = 16;
static_assert(kMaxBlockSize % kConvBlock == 0,
              "the audio block must be a multiple of the convolver block");

static constexpr ConvEngine::Config MakeConvConfig()
{
    ConvEngine::Config c;
    c.block_size       = kConvBlock;
    c.max_ir_len       = kMaxIrLen;
    c.allow_background = false; // cabinets only: no main-loop stage
    c.get_cycles       = daisyconvol_daisy::CycleCount;
    return c;
}
static constexpr ConvEngine::Config kConvConfig = MakeConvConfig();
static_assert(ConvEngine::FastFloats(kConvConfig) > 0, "invalid convolver config");
static_assert(ConvEngine::SlowFloats(kConvConfig) == 0, "cabinet IRs need no SDRAM");

alignas(32) static NAM_DTCM float conv_fast[ConvEngine::FastFloats(kConvConfig)];
alignas(32) static float conv_medium[ConvEngine::MediumFloats(kConvConfig)];
alignas(32) static NAM_DTCM uint8_t
                  conv_fft_tables[ConvEngine::FftTableBytes(kConvConfig)];
static ConvEngine conv;
static bool       conv_ready  = false; // Init succeeded
static volatile bool  g_ir_loaded = false; // block 2 is a cabinet IR
static volatile float g_ir_gain   = 1.0f;  // gain staging, see IrGain

// ── Noise gate ────────────────────────────────────────────────────────────
//
// Power-domain envelope follower with hold-time state machine, matching the
// plugin's Trigger/Gain design but without per-sample log10/pow:
//   - Threshold pre-converted to power in the control loop from the
//     NOISE_GATE_THRESHOLD knob; fully CCW sets threshold to 0 (off).
//   - Gate gain tracked in linear space; no dB conversion per sample.
//   - Operates on the raw input (before the gain knob) so the threshold is
//     gain-independent.
//   - Signal chain: input → [gate] → × gain → NAM → [NAM 2 | IR] → EQ → output.

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

// ── Capture metadata and gain staging ─────────────────────────────────────
//
// .namb metadata block (format v1, bytes 32..79 of the file):
//   32 u8 version major, 33 minor, 34 patch
//   35 u8 flags            0x01 loudness, 0x02 input level, 0x04 output level
//   36 f64 sample_rate
//   44 f64 loudness         dB, as measured by the trainer
//   52 f64 input_level_dbu  "send level": dBu at the gear for 0 dBFS
//   60 f64 output_level_dbu "return level": dBu that produced 0 dBFS back
//   68 reserved[12]
static constexpr size_t  kNambMetaSize      = 80;
static constexpr size_t  kNambMetaFlagsOff  = 35;
static constexpr size_t  kNambLoudnessOff   = 44;
static constexpr size_t  kNambInputDbuOff   = 52;
static constexpr size_t  kNambOutputDbuOff  = 60;
static constexpr uint8_t kMetaHasLoudness   = 0x01;
static constexpr uint8_t kMetaHasInputDbu   = 0x02;
static constexpr uint8_t kMetaHasOutputDbu  = 0x04;

struct CaptureMeta
{
    bool  has_loudness = false;
    bool  has_input    = false;
    bool  has_output   = false;
    float loudness     = 0.0f; // dB
    float input_dbu    = 0.0f;
    float output_dbu   = 0.0f;
};
static CaptureMeta nam_meta[kNumNamBlocks];

static float ReadF64(const uint8_t* p)
{
    double v;
    memcpy(&v, p, sizeof(v));
    return (float)v;
}

// Pull the gain-staging fields out of a .namb, applying the same sanity
// ranges as the plugin: a flagged value outside them counts as absent.
static CaptureMeta ReadCaptureMeta(const uint8_t* data, uint32_t size)
{
    CaptureMeta m;
    if(size < kNambMetaSize)
        return m;
    const uint8_t flags = data[kNambMetaFlagsOff];
    if(flags & kMetaHasLoudness)
    {
        m.loudness     = ReadF64(data + kNambLoudnessOff);
        m.has_loudness = std::isfinite(m.loudness) && m.loudness > -100.0f
                         && m.loudness <= 0.0f;
    }
    if(flags & kMetaHasInputDbu)
    {
        m.input_dbu = ReadF64(data + kNambInputDbuOff);
        m.has_input = std::isfinite(m.input_dbu) && m.input_dbu >= -60.0f
                      && m.input_dbu <= 60.0f;
    }
    if(flags & kMetaHasOutputDbu)
    {
        m.output_dbu = ReadF64(data + kNambOutputDbuOff);
        m.has_output = std::isfinite(m.output_dbu) && m.output_dbu >= -60.0f
                       && m.output_dbu <= 60.0f;
    }
    return m;
}

static constexpr float kTargetLoudnessDb = -18.0f;
static constexpr float kMaxNormalizeDb   = 12.0f;

static inline float DbToGain(float db)
{
    return powf(10.0f, db / 20.0f);
}

// Input calibration: hit the capture at the level its creator used.
static float InputCalibrationDb(const CaptureMeta& m)
{
    return m.has_input ? (kInputLevelDbu - m.input_dbu) : 0.0f;
}

// Loudness normalization: capture loudness -> target, clamped to ±12 dB.
static float NormalizeDb(const CaptureMeta& m)
{
    if(!m.has_loudness)
        return 0.0f;
    float db = kTargetLoudnessDb - m.loudness;
    return fmaxf(-kMaxNormalizeDb, fminf(kMaxNormalizeDb, db));
}

// Compute every NAM block's pre/post gain for the loaded preset. Called from
// LoadPreset once both blocks are known, before audio resumes.
//
// Post-model gain is calibrated hand-off OR normalization, never both (as
// in the plugin): the hand-off applies only to a NAM feeding another NAM
// and only when it carries output_level_dbu; the last NAM block always
// normalizes, so overall level doesn't swing with each capture's metadata.
static void StageChain()
{
    const int last_nam = nam2_loaded ? 1 : 0;
    for(int slot = 0; slot <= last_nam; slot++)
    {
        const CaptureMeta& m   = nam_meta[slot];
        const float        pre = InputCalibrationDb(m);
        float              post;
        const char*        post_why;
        if(slot < last_nam && m.has_output)
        {
            post     = m.output_dbu - kInputLevelDbu; // no clamp: physical level
            post_why = "hand-off";
        }
        else
        {
            post     = NormalizeDb(m);
            post_why = m.has_loudness ? "normalize" : "normalize (no loudness: unity)";
        }
        nam_pre_gain[slot]  = DbToGain(pre);
        nam_post_gain[slot] = DbToGain(post);
        (void)post_why; // only read by LOG
        LOG("  NAM %d staging: in %+.1f dB (%s)  out %+.1f dB (%s)",
            slot + 1,
            pre,
            m.has_input ? "calibrated" : "no input level: unity",
            post,
            post_why);
    }
}

// Cabinet IR gain: unity in-band.
//
// The kernel is scaled so its mean power response over the guitar band
// (100 Hz – 5 kHz, log-spaced, i.e. pink-weighted) is 0 dB. A cab then
// leaves the loudness of the normalized NAM block essentially unchanged, so
// a NAM-only preset and a NAM -> IR preset sit at the same level and the
// -18 dB loudness target holds through the whole chain.
//
// This deliberately differs from the plugin's IR block (unit-energy
// normalization followed by a fixed -18 dB cab pad). A unit-energy cab
// kernel has ~+7 dB of in-band gain (its energy is concentrated in the
// bottom fifth of the spectrum), so the plugin's pad lands the block ~11 dB
// below unity, which its ±24 dB block / main Out knobs absorb. The pedal's
// OUTPUT only attenuates, so that convention would leave every IR preset
// ~10 dB under its NAM-only neighbours with no way to make it up.
//
// Measured directly: a 64-point DFT of the kernel at log-spaced frequencies
// (256K MACs for a 4096-sample IR, a few ms once per preset load). Applied
// to the convolver's output, one multiply per sample.
static constexpr float kIrBandLoHz   = 100.0f;
static constexpr float kIrBandHiHz   = 5000.0f;
static constexpr int   kIrBandPoints = 64;
static constexpr float kIrGainMaxDb  = 24.0f; // sanity clamp either way

static float IrGain(const float* taps, uint32_t n, float sample_rate)
{
    double acc = 0.0;
    for(int k = 0; k < kIrBandPoints; k++)
    {
        const double f = kIrBandLoHz
                         * pow((double)(kIrBandHiHz / kIrBandLoHz),
                               (double)k / (kIrBandPoints - 1));
        const double w  = 2.0 * M_PI * f / sample_rate;
        const double cw = cos(w), sw = sin(w);
        // Recurrence for cos(w·i), sin(w·i): no per-tap transcendental.
        double c = 1.0, s = 0.0, re = 0.0, im = 0.0;
        for(uint32_t i = 0; i < n; i++)
        {
            re += taps[i] * c;
            im += taps[i] * s;
            const double c2 = c * cw - s * sw;
            s               = s * cw + c * sw;
            c               = c2;
        }
        acc += re * re + im * im;
    }
    const double mean_h2 = acc / kIrBandPoints;
    float        db      = 0.0f;
    if(mean_h2 > 0.0 && std::isfinite(mean_h2))
        db = (float)(-10.0 * log10(mean_h2));
    db = fmaxf(-kIrGainMaxDb, fminf(kIrGainMaxDb, db));
    LOG("  IR staging: in-band (%.0f-%.0f Hz) gain %+.1f dB -> normalize %+.1f dB",
        kIrBandLoHz,
        kIrBandHiHz,
        (float)(10.0 * log10(mean_h2)),
        db);
    return DbToGain(db);
}

// ── Model loading ─────────────────────────────────────────────────────────

// Validate a .namb capture in place and load it into NAM block `slot`: parse
// the container (zero-copy: weights point into XIP QSPI), check its CRC,
// confirm it is an A2-Lite capture, pack the weights, and bind + clear the
// instance.  Nothing is written into the engine unless every check passes.
// Does not prewarm.  Must not run while the audio callback is processing:
// callers clear model_loaded first.
static bool LoadCapture(int slot, const uint8_t* data, uint32_t size)
{
    namb_model_t  m;
    namb_status_t st = namb_parse(data, size, &m);
    if(st == NAMB_OK)
        st = namb_verify_crc(data, size);
    if(st == NAMB_OK)
        st = namb_check_a2lite(&m);
    if(st != NAMB_OK)
    {
        LOG("  namb: %s (not an A2-Lite .namb?)", namb_status_str(st));
        return false;
    }
    LOG("  weights: count=%lu  channels=%u  layers=%u",
        (unsigned long)m.weight_count,
        (unsigned)m.channels,
        (unsigned)m.num_layers);

    if(nam_a2lite_load_weights(&nam_weights[slot], m.weights, (int)m.weight_count) != 0)
    {
        LOG("  nam_a2lite_load_weights failed (weight count mismatch)");
        return false;
    }
    nam_a2lite_init(&nam_inst[slot], &nam_weights[slot], &nam_state[slot]);
    nam_meta[slot] = ReadCaptureMeta(data, size);
    LOG("  meta: loudness %s%.1f dB  in %s%.1f dBu  out %s%.1f dBu",
        nam_meta[slot].has_loudness ? "" : "(none) ",
        nam_meta[slot].loudness,
        nam_meta[slot].has_input ? "" : "(none) ",
        nam_meta[slot].input_dbu,
        nam_meta[slot].has_output ? "" : "(none) ",
        nam_meta[slot].output_dbu);
    return true;
}

// Load a cabinet IR from the bank into the convolver (computes its partition
// spectra and clears the engine's delay lines). Audio must not be calling
// IrProcess meanwhile: LoadPreset runs with the chain bypassed.
static bool LoadIr(uint32_t ir_offset, uint32_t ir_num_samples)
{
    uint32_t taps = ir_num_samples;
    if(taps > kMaxIrLen)
    {
        LOG("  IR: %lu taps truncated to %lu",
            (unsigned long)taps,
            (unsigned long)kMaxIrLen);
        taps = kMaxIrLen;
    }

    // IR float32 data sits directly in XIP QSPI (4-byte aligned by packer).
    const float* ir_src
        = reinterpret_cast<const float*>(MODEL_BANK_ADDR + ir_offset);

    if(!conv_ready || !conv.SetIR(ir_src, taps))
    {
        LOG("  IR: rejected (convolver not ready, or non-finite samples)");
        return false;
    }
    g_ir_gain = IrGain(ir_src, taps, hw.AudioSampleRate());
    LOG("  IR: %lu taps (%.1f ms)", (unsigned long)taps, (float)taps / 48.0f);
    return true;
}

// Load the preset at the given index from the QSPI bank.
// Returns true on success; on failure the audio callback stays in bypass.
static bool LoadPreset(uint32_t idx)
{
    const ModelBankHeader* bank
        = reinterpret_cast<const ModelBankHeader*>(MODEL_BANK_ADDR);

    if(bank->magic != MODEL_BANK_MAGIC)
    {
        LOG("  no preset bank at 0x%08lX (magic=0x%08lX)",
            (unsigned long)MODEL_BANK_ADDR,
            (unsigned long)bank->magic);
        return false;
    }
    if(bank->version != MODEL_BANK_VERSION)
    {
        LOG("  bank version %lu, firmware needs %lu: reflash the presets",
            (unsigned long)bank->version,
            (unsigned long)MODEL_BANK_VERSION);
        return false;
    }
    if(bank->num_presets == 0)
    {
        LOG("  preset bank is empty");
        return false;
    }

    if(idx >= bank->num_presets)
        idx = 0;

    const PresetEntry* entries = reinterpret_cast<const PresetEntry*>(
        reinterpret_cast<const uint8_t*>(bank) + sizeof(ModelBankHeader));
    const PresetEntry& e = entries[idx];

    // Bank data in QSPI is 4-byte aligned: MODEL_BANK_ADDR is aligned, the
    // header and entries are multiples of 4 bytes, and the packer pads blobs
    // to 4-byte boundaries.
    const uint8_t* base = reinterpret_cast<const uint8_t*>(MODEL_BANK_ADDR);

    LOG("  preset %lu/%lu: \"%s\"",
        (unsigned long)(idx + 1),
        (unsigned long)bank->num_presets,
        e.name);

    // The companion app always writes 4 entries (one per rotary position)
    // and marks unassigned presets with a zero-size block 1. Treat those as
    // "no model": the caller leaves model_loaded=false and audio passes through.
    if(e.nam_size == 0)
    {
        LOG("  preset is empty (no model assigned)");
        return false;
    }

    // ── Block 1: NAM ──────────────────────────────────────────────────────
    LOG("  NAM: %lu bytes", (unsigned long)e.nam_size);
    if(!LoadCapture(0, base + e.nam_offset, e.nam_size))
        return false;

    // ── Block 2: NAM or cabinet IR ────────────────────────────────────────
    // Clear both regardless of what the new preset carries.
    nam2_loaded = false;
    g_ir_loaded = false;

    if(e.nam2_size != 0)
    {
        LOG("  NAM 2: %lu bytes", (unsigned long)e.nam2_size);
        if(!LoadCapture(1, base + e.nam2_offset, e.nam2_size))
            return false;
        nam2_loaded = true;
    }
    else if(e.ir_offset != 0 && e.ir_num_samples > 0)
    {
        if(!LoadIr(e.ir_offset, e.ir_num_samples))
            return false;
        g_ir_loaded = true;
    }
    else
    {
        LOG("  block 2: none");
    }

    StageChain();

    LOG("  prewarming (%d samples)...", NAM_A2LITE_PREWARM_SAMPLES);
#ifdef LOGGING
    uint32_t t0 = System::GetNow();
#endif
    nam_a2lite_prewarm(&nam_inst[0]);
    if(nam2_loaded)
        nam_a2lite_prewarm(&nam_inst[1]);
    LOG("  preset ready (%lu ms)", (unsigned long)(System::GetNow() - t0));
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

static uint32_t GetBankNumPresets()
{
    const ModelBankHeader* bank
        = reinterpret_cast<const ModelBankHeader*>(MODEL_BANK_ADDR);
    if(bank->magic != MODEL_BANK_MAGIC || bank->version != MODEL_BANK_VERSION)
        return 0;
    return bank->num_presets;
}

// Answer the 'B' serial command: the bank directory as text, one preset per
// line, between fixed begin/end markers so a reader can find it in a stream
// that may also carry LOGGING output. Always available (StartLog is called
// unconditionally). The name is the last field so it needs no escaping.
//
//   T3K-BANK-BEGIN
//   version=2 presets=4 current=2
//   preset=1 nam=1 nam_id=476289 block2=ir block2_id=1234 name=Plexi
//   preset=2 nam=0 nam_id=0 block2=none block2_id=0 name=(empty)
//   ...
//   T3K-BANK-END
//
// A missing or unreadable bank reports `error=...` in place of the entries.
static void PrintBankDirectory()
{
    const ModelBankHeader* bank
        = reinterpret_cast<const ModelBankHeader*>(MODEL_BANK_ADDR);
    hw.PrintLine("T3K-BANK-BEGIN");
    if(bank->magic != MODEL_BANK_MAGIC)
        hw.PrintLine("error=no bank");
    else if(bank->version != MODEL_BANK_VERSION)
        hw.PrintLine("error=bank version %lu, firmware needs %lu",
                     (unsigned long)bank->version,
                     (unsigned long)MODEL_BANK_VERSION);
    else
    {
        hw.PrintLine("version=%lu presets=%lu current=%d",
                     (unsigned long)bank->version,
                     (unsigned long)bank->num_presets,
                     current_preset + 1);
        const PresetEntry* entries = reinterpret_cast<const PresetEntry*>(
            reinterpret_cast<const uint8_t*>(bank) + sizeof(ModelBankHeader));
        for(uint32_t i = 0; i < bank->num_presets && i < 16; i++) // 16: sanity cap
        {
            const PresetEntry& e = entries[i];
            char               name[sizeof(e.name) + 1];
            memcpy(name, e.name, sizeof(e.name));
            name[sizeof(e.name)] = '\0';
            const char* block2 = e.nam2_size != 0 ? "nam"
                                 : (e.ir_offset != 0 && e.ir_num_samples > 0) ? "ir"
                                                                               : "none";
            hw.PrintLine("preset=%lu nam=%d nam_id=%lu block2=%s block2_id=%lu name=%s",
                         (unsigned long)(i + 1),
                         e.nam_size != 0 ? 1 : 0,
                         (unsigned long)e.nam_model_id,
                         block2,
                         (unsigned long)e.block2_model_id,
                         name);
        }
    }
    hw.PrintLine("T3K-BANK-END");
}

static void SwitchToPreset(int idx)
{
    model_loaded = false; // audio falls through to bypass while we reload
    System::Delay(3);     // let an in-flight callback finish with the engine
    LOG("Switching to preset %d", idx + 1);
    model_loaded = LoadPreset((uint32_t)idx);
    LOG("Preset load: %s", model_loaded ? "OK" : "FAILED");
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
// The convolver runs kConvBlock-sample blocks, so a callback's block is fed
// through it in slices; zero added latency either way. src and dst may alias.

static void IrProcess(const float* src, float* dst, uint32_t block_size)
{
    for(uint32_t i = 0; i < block_size; i += kConvBlock)
        conv.ProcessBlock(src + i, dst + i, kConvBlock);
    const float g = g_ir_gain;
    for(uint32_t i = 0; i < block_size; i++)
        dst[i] *= g;
}

// ── Signal chain ──────────────────────────────────────────────────────────

// Run the loaded preset on mono_in (gated, INPUT-knob-scaled input) and
// return the buffer holding the result (mono_out or mono_in).  The two
// buffers are ping-ponged so no block is ever copied:
//   NAM 1:  mono_in  → mono_out
//   NAM 2:  mono_out → mono_in   (block 2 as NAM; mono_in is free by then)
//   IR:     mono_out → mono_in   (block 2 as IR; never together with NAM 2)
//   EQ:     in place, after the whole block chain
// Each NAM block is wrapped in its pre gain (input calibration) and post
// gain (calibrated hand-off or loudness normalize); the IR's gain staging is
// applied inside IrProcess. See StageChain / IrGain.
static inline void NamBlock(int slot, float* in, float* out, size_t num_frames)
{
    const float pre  = nam_pre_gain[slot];
    const float post = nam_post_gain[slot];
    if(pre != 1.0f)
        for(size_t i = 0; i < num_frames; i++)
            in[i] *= pre;
    nam_a2lite_process(&nam_inst[slot],
                       (const float* const*)&in,
                       &out,
                       (int)num_frames);
    if(post != 1.0f)
        for(size_t i = 0; i < num_frames; i++)
            out[i] *= post;
}

static inline const float* ProcessChain(size_t num_frames)
{
    NamBlock(0, mono_in, mono_out, num_frames);
    float* out = mono_out;

    // Block 2: second capture or cabinet IR, each mono_out → mono_in.
    if(nam2_loaded)
    {
        NamBlock(1, mono_out, mono_in, num_frames);
        out = mono_in;
    }
    else if(g_ir_loaded)
    {
        IrProcess(mono_out, mono_in, num_frames);
        out = mono_in;
    }

    // Tone stack on the finished chain.
    for(size_t i = 0; i < num_frames; i++)
        out[i] = eq.Process(out[i]);
    return out;
}

// ── Audio callback ────────────────────────────────────────────────────────

// Whole-callback cycle counts against the 1 ms block deadline (48 frames at
// 48 kHz, 480,000 cycles at 480 MHz). An overrun is audible as a dropout.
static constexpr uint32_t kBlockBudgetCycles = 480000;
static volatile uint32_t  cb_count           = 0;
static volatile uint32_t  cb_cycles          = 0; // last block
static volatile uint32_t  cb_max_cycles      = 0;
static volatile uint32_t  cb_overruns        = 0;

static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    const uint32_t cyc0 = DWT->CYCCNT;
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

        const float* chain_out = ProcessChain(num_frames);

        for(size_t i = 0; i < num_frames; i++)
        {
            float s = chain_out[i] * volume;
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

    const uint32_t elapsed = DWT->CYCCNT - cyc0;
    cb_cycles              = elapsed;
    if(elapsed > cb_max_cycles)
        cb_max_cycles = elapsed;
    if(elapsed > kBlockBudgetCycles)
        cb_overruns++;
}

// ── Entry point ───────────────────────────────────────────────────────────

// The engines' code (NAM, convolver, CMSIS FFT) is linked into .itcmram_text
// (ITCM, zero wait states) but stored in QSPI flash; the Makefile's derived
// linker script defines these symbols. The copy runs from .preinit_array,
// i.e. before any C++ static initializer: some of those live in ITCM too
// (daisyConvol's FFT backend has one), and would otherwise run from
// uninitialized ITCM and fault before main().
extern "C" uint32_t _sitcmram_text, _itcmram_text_start, _itcmram_text_end;

static void CopyCodeToItcm()
{
    const volatile uint32_t* src = &_sitcmram_text;
    volatile uint32_t*       dst = &_itcmram_text_start;
    while(dst < &_itcmram_text_end)
        *dst++ = *src++;
    __DSB();
    __ISB();
}
__attribute__((section(".preinit_array"), used)) static void (*const
    copy_code_to_itcm_entry)() = CopyCodeToItcm;

int main(void)
{
    // Boost = 480 MHz (libDaisy's default is 400 MHz). Two chained A2-Lite
    // captures need it: they take ~83% of the block at 480 MHz.
    hw.Init(true);

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
    LOG("T3K Pedal (A2-Lite engine): booting at %lu MHz",
        (unsigned long)(System::GetSysClkFreq() / 1000000));
#endif

    hw.SetAudioBlockSize(kMaxBlockSize);

    // Cabinet convolver: lays out (and zeroes) its arenas and FFT plans, then
    // moves the FFT twiddle / bit-reversal tables from QSPI into DTCM.
    // The DWT cycle counter (enabled further down) feeds conv.perf().
    conv_ready = conv.Init(kConvConfig,
                           {conv_fast, ConvEngine::FastFloats(kConvConfig), 0},
                           {conv_medium, ConvEngine::MediumFloats(kConvConfig), 0},
                           {})
                 && daisyconvol::FftRelocateTables(conv_fft_tables,
                                                   sizeof(conv_fft_tables));
    LOG("Convolver: %s (block %u, IR up to %lu samples)",
        conv_ready ? "ready" : "INIT FAILED",
        (unsigned)kConvBlock,
        (unsigned long)kMaxIrLen);
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
                 GPIO::Pull::PULLUP);
    rotary2.Init(hw.GetPin(T3kPedal::ROTARY_2),
                 0.f,
                 Switch::TYPE_TOGGLE,
                 Switch::POLARITY_INVERTED,
                 GPIO::Pull::PULLUP);
    rotary3.Init(hw.GetPin(T3kPedal::ROTARY_3),
                 0.f,
                 Switch::TYPE_TOGGLE,
                 Switch::POLARITY_INVERTED,
                 GPIO::Pull::PULLUP);
    rotary4.Init(hw.GetPin(T3kPedal::ROTARY_4),
                 0.f,
                 Switch::TYPE_TOGGLE,
                 Switch::POLARITY_INVERTED,
                 GPIO::Pull::PULLUP);
    led_status.Init(hw.GetPin(T3kPedal::LED_STATUS), GPIO::Mode::OUTPUT);
    led_preset[0].Init(hw.GetPin(T3kPedal::LED_PRESET_1), GPIO::Mode::OUTPUT);
    led_preset[1].Init(hw.GetPin(T3kPedal::LED_PRESET_2), GPIO::Mode::OUTPUT);
    led_preset[2].Init(hw.GetPin(T3kPedal::LED_PRESET_3), GPIO::Mode::OUTPUT);
    led_preset[3].Init(hw.GetPin(T3kPedal::LED_PRESET_4), GPIO::Mode::OUTPUT);

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

    LOG("Loading preset from bank at 0x%08lX", (unsigned long)MODEL_BANK_ADDR);
    model_loaded = LoadPreset((uint32_t)current_preset);
    LOG("Preset load: %s", model_loaded ? "OK" : "FAILED");
    UpdatePresetLeds(current_preset);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#ifdef LOGGING
    // One-shot benchmark of the loaded preset's chain against the block budget.
    if(model_loaded)
    {
        constexpr int kRuns = 100;
        uint32_t      sum = 0, max = 0;
        for(int run = 0; run < kRuns; run++)
        {
            memset(mono_in, 0, sizeof(mono_in));
            const uint32_t t0 = DWT->CYCCNT;
            ProcessChain(kMaxBlockSize);
            const uint32_t cy = DWT->CYCCNT - t0;
            sum += cy;
            if(cy > max)
                max = cy;
        }
        const uint32_t avg = sum / kRuns;
        LOG("Benchmark %s%s (%d x %u frames, budget %lu cy): avg %lu (%.1f%%)  max %lu (%.1f%%)",
            nam2_loaded ? "NAM + NAM" : "NAM",
            g_ir_loaded ? " + IR" : "",
            kRuns,
            (unsigned)kMaxBlockSize,
            (unsigned long)kBlockBudgetCycles,
            (unsigned long)avg,
            100.0f * (float)avg / kBlockBudgetCycles,
            (unsigned long)max,
            100.0f * (float)max / kBlockBudgetCycles);
    }
#endif

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

        if(bank_requested)
        {
            bank_requested = false;
            PrintBankDirectory();
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
        // position actually exists in the bank.
        int rotary_preset = ReadRotaryPreset();
        if(rotary_preset >= 0 && rotary_preset != current_preset
           && (uint32_t)rotary_preset < GetBankNumPresets())
            SwitchToPreset(rotary_preset);

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
            LOG("cb=%lu  cycles=%lu (%.1f%%)  max=%lu (%.1f%%)  overruns=%lu  %s  preset=%d",
                (unsigned long)cb_count,
                (unsigned long)cb_cycles,
                100.0f * (float)cb_cycles / kBlockBudgetCycles,
                (unsigned long)cb_max_cycles,
                100.0f * (float)cb_max_cycles / kBlockBudgetCycles,
                (unsigned long)cb_overruns,
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
            if(g_ir_loaded)
            {
                // Per 16-sample engine block; x3 per audio callback.
                const daisyconvol::Perf p = conv.perf();
                LOG("  IR conv: avg %lu cy  peak %lu cy  (x3 per callback)"
                    "  nonfinite=%lu",
                    (unsigned long)p.callback.Avg(),
                    (unsigned long)p.callback.peak,
                    (unsigned long)p.nonfinite_inputs);
            }
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
