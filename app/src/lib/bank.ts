// Model bank builder, port of pack_models.py build_bank().
//
// Bank layout (little-endian):
//   Header (64 B): u32 magic "NAMP", u32 version, u32 num_models, padding
//   Entries (48 B each): name[32], u32 namb_offset, u32 namb_size,
//                        u32 ir_offset (0 = none), u32 ir_num_samples
//   Blobs: raw .namb bytes then optional IR float32[], each 4-byte aligned.
// Offsets are absolute from bank start.

export const BANK_MAGIC = 0x504d414e; // "NAMP"
export const BANK_VERSION = 1;
export const HEADER_SIZE = 64;
export const ENTRY_SIZE = 48;
export const MAX_IR_TAPS = 256; // must match MAX_IR_TAPS in NAMPedal.cpp

// QSPI flash: bank lives at 0x90600000. The Daisy bootloader's writable QSPI
// region ends at 0x907BFFFF (layout "@Flash /0x90000000/64*4Kg/0x90040000/
// 60*64Kg/0x90400000/60*64Kg"), leaving 1.75 MB for the bank.
export const MODEL_BANK_ADDR = 0x90600000;
export const MODEL_BANK_MAX_SIZE = 0x1c0000; // 1.75 MB

export interface BankModel {
  name: string; // display name, max 31 ASCII chars
  namb: Uint8Array;
  ir: Float32Array | null;
}

const align4 = (n: number) => (n + 3) & ~3;

export function buildBank(models: BankModel[]): Uint8Array {
  const n = models.length;
  const dataBase = HEADER_SIZE + n * ENTRY_SIZE;

  // Compute blob offsets (namb then optional IR per model, 4-byte aligned)
  const nambOffsets: number[] = [];
  const irOffsets: number[] = [];
  let cur = dataBase;
  for (const m of models) {
    nambOffsets.push(cur);
    cur = align4(cur + m.namb.length);
    if (m.ir) {
      irOffsets.push(cur);
      cur = align4(cur + m.ir.length * 4);
    } else {
      irOffsets.push(0);
    }
  }

  const out = new Uint8Array(cur);
  const view = new DataView(out.buffer);

  // Header
  view.setUint32(0, BANK_MAGIC, true);
  view.setUint32(4, BANK_VERSION, true);
  view.setUint32(8, n, true);

  // Entries
  const enc = new TextEncoder();
  for (let i = 0; i < n; i++) {
    const base = HEADER_SIZE + i * ENTRY_SIZE;
    // ASCII name, max 31 chars + NUL padding (non-ASCII replaced)
    const ascii = models[i].name.replace(/[^\x20-\x7e]/g, '?').slice(0, 31);
    out.set(enc.encode(ascii), base);
    view.setUint32(base + 32, nambOffsets[i], true);
    view.setUint32(base + 36, models[i].namb.length, true);
    view.setUint32(base + 40, irOffsets[i], true);
    view.setUint32(base + 44, models[i].ir ? models[i].ir!.length : 0, true);
  }

  // Blobs
  for (let i = 0; i < n; i++) {
    out.set(models[i].namb, nambOffsets[i]);
    const ir = models[i].ir;
    if (ir) {
      for (let j = 0; j < ir.length; j++) {
        view.setFloat32(irOffsets[i] + j * 4, ir[j], true);
      }
    }
  }

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

function trimTaper(samples: Float32Array, nTaps: number, fadeLen = 64): Float32Array {
  const out = new Float32Array(nTaps);
  out.set(samples.subarray(0, Math.min(samples.length, nTaps)));
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
  s = trimTaper(s, MAX_IR_TAPS, 64);
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
