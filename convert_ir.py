#!/usr/bin/env python3
"""
convert_ir.py — prepare a cabinet IR WAV for the T3K Pedal.

Pipeline:
  1. Resample to --rate if needed (scipy polyphase > numpy > pure-Python)
  2. Minimum-phase conversion via real cepstrum (requires numpy)
     Front-loads IR energy so truncation at --taps loses minimal content.
  3. Truncate to --taps; apply a Hann half-window over the last --fade samples
     to prevent spectral ripple from the hard cut-off.
  4. Peak-normalise to 1.0.

Usage:
  python convert_ir.py INPUT.wav OUTPUT.wav [options]

  --taps N        Target length in samples (default: 4096, matches kMaxIrLen)
  --rate HZ       Target sample rate in Hz (default: 48000)
  --fade N        Hann tail applied to the last N samples (default: 256, ~5 ms)
  --no-min-phase  Skip minimum-phase conversion (not recommended)
  --no-normalize  Skip peak normalisation
  --eps FLOAT     Log-magnitude floor for min-phase cepstrum, relative to peak
                  (default: 1e-8; lower = more accurate, higher = more stable)

The output is a mono IEEE-float-32 WAV ready for pack_models.py:

  python pack_models.py --flash "Amp:model.namb:OUTPUT.wav"
"""

import argparse
import math
import os
import struct
import sys


# ── WAV I/O ───────────────────────────────────────────────────────────────

def read_wav_mono_f32(path):
    """Return (samples: list[float], sample_rate: int).

    Supports PCM 16/24/32-bit and IEEE float 32-bit.
    Multi-channel files use channel 0 only.
    """
    try:
        raw = open(path, 'rb').read()
    except OSError as e:
        sys.exit(f'error: {e}')

    if raw[:4] != b'RIFF' or raw[8:12] != b'WAVE':
        sys.exit(f'error: {path}: not a WAV file')

    fmt_chunk = data_chunk = None
    pos = 12
    while pos + 8 <= len(raw):
        tag  = raw[pos:pos + 4]
        clen = struct.unpack_from('<I', raw, pos + 4)[0]
        body = raw[pos + 8: pos + 8 + clen]
        if tag == b'fmt ':
            fmt_chunk = body
        elif tag == b'data':
            data_chunk = body
        pos += 8 + clen + (clen & 1)

    if fmt_chunk is None or data_chunk is None:
        sys.exit(f'error: {path}: missing fmt or data chunk')

    audio_fmt   = struct.unpack_from('<H', fmt_chunk, 0)[0]
    num_ch      = struct.unpack_from('<H', fmt_chunk, 2)[0]
    sample_rate = struct.unpack_from('<I', fmt_chunk, 4)[0]
    bps         = struct.unpack_from('<H', fmt_chunk, 14)[0]

    if audio_fmt == 0xFFFE and len(fmt_chunk) >= 26:
        audio_fmt = struct.unpack_from('<H', fmt_chunk, 24)[0]

    frame_bytes = (bps // 8) * num_ch

    def s24(d, off):
        v = d[off] | (d[off+1] << 8) | (d[off+2] << 16)
        return v - 0x1000000 if v >= 0x800000 else v

    n = len(data_chunk) // frame_bytes
    if   audio_fmt == 3 and bps == 32:
        raw_s = list(struct.unpack_from(f'<{n * num_ch}f', data_chunk))
    elif audio_fmt == 1 and bps == 16:
        raw_s = [s / 32768.0     for s in struct.unpack_from(f'<{n * num_ch}h', data_chunk)]
    elif audio_fmt == 1 and bps == 24:
        raw_s = [s24(data_chunk, i * 3) / 8388608.0 for i in range(n * num_ch)]
    elif audio_fmt == 1 and bps == 32:
        raw_s = [s / 2147483648.0 for s in struct.unpack_from(f'<{n * num_ch}i', data_chunk)]
    else:
        sys.exit(f'error: {path}: unsupported format {audio_fmt}/{bps}-bit')

    return raw_s[::num_ch], sample_rate


def write_wav_f32_mono(path, samples, sample_rate):
    n    = len(samples)
    data = struct.pack(f'<{n}f', *samples)
    fmt  = struct.pack('<HHIIHH', 3, 1, sample_rate, sample_rate * 4, 4, 32)
    out  = b'RIFF'
    out += struct.pack('<I', 4 + (8 + len(fmt)) + (8 + len(data)))
    out += b'WAVE'
    out += b'fmt '  + struct.pack('<I', len(fmt))  + fmt
    out += b'data'  + struct.pack('<I', len(data)) + data
    with open(path, 'wb') as f:
        f.write(out)


# ── Resampling ────────────────────────────────────────────────────────────

def resample_to(samples, from_rate, to_rate):
    if from_rate == to_rate:
        return samples

    try:
        from scipy.signal import resample_poly
        from math import gcd
        import numpy as np
        g = gcd(to_rate, from_rate)
        up, down = to_rate // g, from_rate // g
        print(f'  scipy polyphase  up={up}  down={down}')
        return list(resample_poly(np.array(samples, dtype='float64'), up, down))
    except ImportError:
        pass

    try:
        import numpy as np
        n_out = round(len(samples) * to_rate / from_rate)
        old_t = np.linspace(0.0, 1.0, len(samples))
        new_t = np.linspace(0.0, 1.0, n_out)
        print('  numpy linear interpolation (install scipy for higher quality)')
        return list(np.interp(new_t, old_t, samples))
    except ImportError:
        pass

    n_in  = len(samples)
    n_out = round(n_in * to_rate / from_rate)
    print('  pure-Python linear interpolation (install numpy/scipy for higher quality)')
    result = []
    for i in range(n_out):
        t  = i * (n_in - 1) / max(n_out - 1, 1)
        lo = int(t)
        hi = min(lo + 1, n_in - 1)
        result.append(samples[lo] + (t - lo) * (samples[hi] - samples[lo]))
    return result


# ── Minimum-phase conversion ──────────────────────────────────────────────

def minimum_phase(samples, eps=1e-8):
    """Convert IR to its minimum-phase equivalent via the real-cepstrum method.

    The minimum-phase version has the same magnitude spectrum but concentrates
    all energy at the start of the impulse response, so truncation at a fixed
    tap count causes far less spectral coloration than truncating a linear-phase
    or mixed-phase IR.

    Algorithm:
      H       = FFT(h, N)                  — N = 4× next power-of-2 above len(h)
      log_mag = log(max(|H|, eps * peak))  — eps floor prevents log singularity
      c       = IFFT(log_mag)              — real cepstrum (symmetric)
      fold    — double c[1..N/2-1], keep c[0] and c[N/2], zero anti-causal half
      H_mp    = exp(FFT(fold))             — minimum-phase spectrum
      h_mp    = IFFT(H_mp)[:len(h)]       — time-domain minimum-phase IR

    The 4× FFT oversize prevents cepstral aliasing (the cepstrum of a long IR
    wraps around if the FFT is too small, corrupting the result).
    The relative eps floor (not absolute) handles spectral nulls without
    significantly altering the shape of the magnitude response.

    Requires numpy. Raises ImportError if not available.
    """
    import numpy as np

    n = len(samples)
    if n < 4:
        return samples  # too short to be meaningful

    # 4× next-power-of-2 to prevent cepstral time-domain aliasing
    fft_size = 1
    while fft_size < n:
        fft_size <<= 1
    fft_size <<= 2

    h    = np.array(samples, dtype=np.float64)
    H    = np.fft.rfft(h, n=fft_size)
    mag  = np.abs(H)
    peak = float(np.max(mag))
    if peak < 1e-30:
        return samples  # silent IR

    # Log-magnitude with floor relative to spectral peak
    log_mag = np.log(np.maximum(mag, eps * peak))

    # Real cepstrum: IFFT of log-magnitude (treated as Hermitian-symmetric)
    cepstrum = np.fft.irfft(log_mag, n=fft_size)

    # Minimum-phase liftering: double causal half, zero anti-causal half
    fold              = np.zeros(fft_size)
    fold[0]           = cepstrum[0]                          # DC
    fold[1:fft_size // 2] = 2.0 * cepstrum[1:fft_size // 2] # causal (doubled)
    fold[fft_size // 2]   = cepstrum[fft_size // 2]         # Nyquist

    # Reconstruct minimum-phase IR
    H_mp = np.exp(np.fft.rfft(fold, n=fft_size))
    h_mp = np.fft.irfft(H_mp, n=fft_size)
    return list(h_mp[:n])


# ── Tail taper ────────────────────────────────────────────────────────────

def trim_and_taper(samples, n_taps, fade_len):
    """Truncate or zero-pad to n_taps, then apply a Hann half-window to the tail.

    The Hann taper (1 → 0 over fade_len samples) prevents the spectral ripple
    caused by the hard discontinuity at the truncation point.  A fade covering
    ~25% of the target length is a good default; longer fades trade off more
    high-frequency content for a smoother magnitude response.
    """
    if len(samples) >= n_taps:
        out = list(samples[:n_taps])
    else:
        out = list(samples) + [0.0] * (n_taps - len(samples))

    if fade_len > 0:
        fade_len = min(fade_len, n_taps)
        start = n_taps - fade_len
        for i in range(fade_len):
            t = i / max(fade_len - 1, 1)
            out[start + i] *= 0.5 * (1.0 + math.cos(math.pi * t))  # 1 → 0

    return out


# ── Normalisation ─────────────────────────────────────────────────────────

def normalize(samples):
    """Scale so the absolute peak equals 1.0.  No-op for a silent signal."""
    peak = max(abs(s) for s in samples)
    if peak < 1e-30:
        return list(samples)
    return [s / peak for s in samples]


# ── Entry point ───────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input',  help='Input IR WAV file')
    ap.add_argument('output', help='Output WAV file (float32 mono)')
    ap.add_argument('--taps', type=int, default=4096,
                    help='Target length in taps (default: 4096)')
    ap.add_argument('--rate', type=int, default=48000,
                    help='Target sample rate in Hz (default: 48000)')
    ap.add_argument('--fade', type=int, default=256,
                    help='Hann tail length in samples (default: 256, ~5 ms)')
    ap.add_argument('--no-min-phase', action='store_true',
                    help='Skip minimum-phase conversion')
    ap.add_argument('--no-normalize', action='store_true',
                    help='Skip peak normalisation')
    ap.add_argument('--eps', type=float, default=1e-8,
                    help='Log-magnitude floor for cepstrum, relative to peak '
                         '(default: 1e-8)')
    args = ap.parse_args()

    samples, sr = read_wav_mono_f32(args.input)
    n_in = len(samples)
    print(f'input:  {os.path.basename(args.input)}'
          f'  {n_in} taps @ {sr} Hz  ({n_in / sr * 1000:.1f} ms)')

    # 1. Resample
    if sr != args.rate:
        print(f'resampling {sr} Hz → {args.rate} Hz ...')
        samples = resample_to(samples, sr, args.rate)
        print(f'  {len(samples)} taps after resample')

    # 2. Minimum-phase conversion
    if not args.no_min_phase:
        try:
            samples = minimum_phase(samples, eps=args.eps)
            print(f'min-phase: applied  (eps={args.eps:.0e})')
        except ImportError:
            print('min-phase: skipped — numpy not found  (pip install numpy)')
    else:
        print('min-phase: skipped (--no-min-phase)')

    # 3. Truncate + Hann tail taper
    n_before = len(samples)
    action   = 'truncated' if n_before > args.taps else 'zero-padded'
    samples  = trim_and_taper(samples, args.taps, args.fade)
    fade_str = f', Hann tail {args.fade} samples' if args.fade > 0 else ''
    print(f'{action} {n_before} → {args.taps} taps'
          f'  ({n_before / args.rate * 1000:.1f} ms → {args.taps / args.rate * 1000:.1f} ms)'
          f'{fade_str}')

    # 4. Normalise
    if not args.no_normalize:
        peak = max(abs(s) for s in samples)
        if peak > 1e-30:
            samples = [s / peak for s in samples]
            print(f'normalized: peak was {20 * math.log10(peak):.1f} dBFS → 0.0 dBFS')
        else:
            print('normalize: signal is silent, skipped')
    else:
        peak = max(abs(s) for s in samples)
        peak_db = 20 * math.log10(peak) if peak > 1e-30 else float('-inf')
        print(f'normalize: skipped  (peak {peak:.5f} = {peak_db:.1f} dBFS)')

    print(f'output: {os.path.basename(args.output)}'
          f'  {args.taps} taps @ {args.rate} Hz  ({args.taps / args.rate * 1000:.1f} ms)')
    write_wav_f32_mono(args.output, samples, args.rate)


if __name__ == '__main__':
    main()
