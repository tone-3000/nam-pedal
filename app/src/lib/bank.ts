// Preset bank builder, port of pack_models.py build_bank().
//
// Bank layout (little-endian), version 2:
//   Header (64 B): u32 magic "NAMP", u32 version, u32 num_presets, padding
//   Entries (64 B each): name[32],
//                        u32 nam_offset,  u32 nam_size   (block 1: A2-Lite .namb; 0 = empty preset)
//                        u32 nam2_offset, u32 nam2_size  (block 2: second .namb, or 0)
//                        u32 ir_offset,   u32 ir_num_samples (block 2: cab IR, or 0)
//                        u32 nam_model_id, u32 block2_model_id (TONE3000 model ids, 0 = unknown)
//   Blobs: raw .namb bytes and IR float32[], each 4-byte aligned.
// Offsets are absolute from bank start.  nam2 and ir are mutually exclusive.
// The model ids let "Sync from pedal" rebuild the app's preset state from
// the pedal's bank directory (firmware 'B' serial command); the firmware
// itself ignores them.

export const BANK_MAGIC = 0x504d414e; // "NAMP"
export const BANK_VERSION = 2;
export const HEADER_SIZE = 64;
export const ENTRY_SIZE = 64;
export const MAX_IR_TAPS = 4096; // must match kMaxIrLen in t3k_pedal.cpp

// QSPI flash: bank lives at 0x90600000. The Daisy bootloader's writable QSPI
// region ends at 0x907BFFFF (layout "@Flash /0x90000000/64*4Kg/0x90040000/
// 60*64Kg/0x90400000/60*64Kg"), leaving 1.75 MB for the bank.
export const MODEL_BANK_ADDR = 0x90600000;
export const MODEL_BANK_MAX_SIZE = 0x1c0000; // 1.75 MB

export interface BankPreset {
  name: string; // display name, max 31 ASCII chars
  namb: Uint8Array; // block 1 (empty = unused preset)
  namb2: Uint8Array | null; // block 2: second capture...
  ir: Float32Array | null; // ...or cab IR
  namModelId?: number; // TONE3000 model id of block 1 (0 / absent = unknown)
  block2ModelId?: number; // TONE3000 model id of block 2, NAM or IR
}

const align4 = (n: number) => (n + 3) & ~3;

export function buildBank(presets: BankPreset[]): Uint8Array {
  const n = presets.length;

  // Lay out the blobs in order (namb, namb2, ir per preset), 4-byte aligned.
  // Each blob is [offset, bytes]; offset 0 = absent.
  type Blob = [number, Uint8Array];
  const toBytes = (ir: Float32Array) => {
    const bytes = new Uint8Array(ir.length * 4);
    const v = new DataView(bytes.buffer);
    ir.forEach((x, i) => v.setFloat32(i * 4, x, true)); // explicit little-endian
    return bytes;
  };
  const layout: Blob[][] = [];
  let cur = HEADER_SIZE + n * ENTRY_SIZE;
  for (const p of presets) {
    const row: Blob[] = [];
    for (const bytes of [p.namb, p.namb2, p.ir && toBytes(p.ir)]) {
      if (!bytes || bytes.length === 0) {
        row.push([0, new Uint8Array(0)]);
        continue;
      }
      row.push([cur, bytes]);
      cur = align4(cur + bytes.length);
    }
    layout.push(row);
  }

  const out = new Uint8Array(cur);
  const view = new DataView(out.buffer);

  // Header
  view.setUint32(0, BANK_MAGIC, true);
  view.setUint32(4, BANK_VERSION, true);
  view.setUint32(8, n, true);

  // Entries + blobs.  The IR size field is a sample count, not bytes.
  const enc = new TextEncoder();
  layout.forEach((row, i) => {
    const base = HEADER_SIZE + i * ENTRY_SIZE;
    // ASCII name, max 31 chars + NUL padding (non-ASCII replaced)
    const ascii = presets[i].name.replace(/[^\x20-\x7e]/g, '?').slice(0, 31);
    out.set(enc.encode(ascii), base);
    row.forEach(([offset, bytes], j) => {
      view.setUint32(base + 32 + j * 8, offset, true);
      view.setUint32(base + 36 + j * 8, j === 2 ? bytes.length / 4 : bytes.length, true);
      if (offset) out.set(bytes, offset);
    });
    view.setUint32(base + 56, presets[i].namModelId ?? 0, true);
    view.setUint32(base + 60, presets[i].block2ModelId ?? 0, true);
  });

  return out;
}

// ─── IR processing pipeline (port of pack_models.py process_ir) ──────────────
// resample → minimum-phase → truncate + Hann tail taper → peak normalize

function resampleLinear(samples: Float32Array, fromRate: number, toRate: number): Float32Array {
  if (fromRate === toRate) return samples;
  const nIn = samples.length;
  const nOut = Math.round((nIn * toRate) / fromRate);
  const out = new Float32Array(nOut);
  for (let i = 0; i < nOut; i++) {
    const t = (i * (nIn - 1)) / Math.max(nOut - 1, 1);
    const lo = Math.floor(t);
    const hi = Math.min(lo + 1, nIn - 1);
    out[i] = samples[lo] + (t - lo) * (samples[hi] - samples[lo]);
  }
  return out;
}

// In-place radix-2 complex FFT (interleaved re/im not used; separate arrays)
function fft(re: Float64Array, im: Float64Array, invert: boolean) {
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      [re[i], re[j]] = [re[j], re[i]];
      [im[i], im[j]] = [im[j], im[i]];
    }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = ((2 * Math.PI) / len) * (invert ? 1 : -1);
    const wRe = Math.cos(ang), wIm = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let curRe = 1, curIm = 0;
      for (let j = 0; j < len / 2; j++) {
        const uRe = re[i + j], uIm = im[i + j];
        const vRe = re[i + j + len / 2] * curRe - im[i + j + len / 2] * curIm;
        const vIm = re[i + j + len / 2] * curIm + im[i + j + len / 2] * curRe;
        re[i + j] = uRe + vRe;
        im[i + j] = uIm + vIm;
        re[i + j + len / 2] = uRe - vRe;
        im[i + j + len / 2] = uIm - vIm;
        const nextRe = curRe * wRe - curIm * wIm;
        curIm = curRe * wIm + curIm * wRe;
        curRe = nextRe;
      }
    }
  }
  if (invert) {
    for (let i = 0; i < n; i++) {
      re[i] /= n;
      im[i] /= n;
    }
  }
}

// Minimum-phase conversion via real cepstrum (port of _ir_minimum_phase)
function minimumPhase(samples: Float32Array, eps = 1e-8): Float32Array {
  const n = samples.length;
  if (n < 4) return samples;
  let fftSize = 1;
  while (fftSize < n) fftSize <<= 1;
  fftSize <<= 2; // 4x to prevent cepstral aliasing

  const re = new Float64Array(fftSize);
  const im = new Float64Array(fftSize);
  for (let i = 0; i < n; i++) re[i] = samples[i];
  fft(re, im, false);

  let peak = 0;
  const mag = new Float64Array(fftSize);
  for (let i = 0; i < fftSize; i++) {
    mag[i] = Math.hypot(re[i], im[i]);
    if (mag[i] > peak) peak = mag[i];
  }
  if (peak < 1e-30) return samples;

  // log magnitude → cepstrum
  const cRe = new Float64Array(fftSize);
  const cIm = new Float64Array(fftSize);
  for (let i = 0; i < fftSize; i++) cRe[i] = Math.log(Math.max(mag[i], eps * peak));
  fft(cRe, cIm, true);

  // Fold cepstrum to make it causal (minimum phase)
  const fRe = new Float64Array(fftSize);
  const fIm = new Float64Array(fftSize);
  fRe[0] = cRe[0];
  for (let i = 1; i < fftSize / 2; i++) fRe[i] = 2 * cRe[i];
  fRe[fftSize / 2] = cRe[fftSize / 2];

  // exp of the spectrum of the folded cepstrum
  fft(fRe, fIm, false);
  for (let i = 0; i < fftSize; i++) {
    const eRe = Math.exp(fRe[i]);
    fRe[i] = eRe * Math.cos(fIm[i]);
    fIm[i] = eRe * Math.sin(fIm[i]);
  }
  fft(fRe, fIm, true);

  const out = new Float32Array(n);
  for (let i = 0; i < n; i++) out[i] = fRe[i];
  return out;
}

// IRs already within nTaps are kept as they are, not zero-padded: the
// firmware's convolver only spends CPU on the partitions an IR occupies.
function trimTaper(samples: Float32Array, nTaps: number, fadeLen = 256): Float32Array {
  if (samples.length <= nTaps) return samples.slice();
  const out = samples.slice(0, nTaps);
  fadeLen = Math.min(fadeLen, nTaps);
  const start = nTaps - fadeLen;
  for (let i = 0; i < fadeLen; i++) {
    const t = i / Math.max(fadeLen - 1, 1);
    out[start + i] *= 0.5 * (1.0 + Math.cos(Math.PI * t));
  }
  return out;
}

export function processIr(samples: Float32Array, sampleRate: number, targetRate = 48000): Float32Array {
  let s = resampleLinear(samples, sampleRate, targetRate);
  s = minimumPhase(s);
  s = trimTaper(s, MAX_IR_TAPS, 256);
  let peak = 0;
  for (let i = 0; i < s.length; i++) peak = Math.max(peak, Math.abs(s[i]));
  if (peak > 1e-30) {
    for (let i = 0; i < s.length; i++) s[i] /= peak;
  }
  return s;
}

// Decode a WAV file (channel 0), resampled by the browser to 48 kHz.
// OfflineAudioContext.decodeAudioData resamples to the context rate, which
// replaces the resample step of the Python pipeline with a higher-quality one.
export async function decodeWavChannel0At48k(buf: ArrayBuffer): Promise<Float32Array> {
  const ctx = new OfflineAudioContext(1, 1, 48000);
  const decoded = await ctx.decodeAudioData(buf.slice(0));
  return decoded.getChannelData(0);
}
