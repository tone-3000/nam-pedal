// .nam (JSON) → .namb (compact binary) converter.
// Port of tone-3000/nam-binary-loader tools/nam2namb.cpp.
// All multi-byte values are little-endian. Format version 1.

// ─── Binary writer ────────────────────────────────────────────────────────────

class BinaryWriter {
  private buf = new Uint8Array(4096);
  private len = 0;

  private ensure(n: number) {
    if (this.len + n <= this.buf.length) return;
    let cap = this.buf.length * 2;
    while (cap < this.len + n) cap *= 2;
    const next = new Uint8Array(cap);
    next.set(this.buf.subarray(0, this.len));
    this.buf = next;
  }

  u8(v: number) { this.ensure(1); this.buf[this.len++] = v & 0xff; }
  u16(v: number) { this.ensure(2); new DataView(this.buf.buffer).setUint16(this.len, v, true); this.len += 2; }
  u32(v: number) { this.ensure(4); new DataView(this.buf.buffer).setUint32(this.len, v >>> 0, true); this.len += 4; }
  i32(v: number) { this.ensure(4); new DataView(this.buf.buffer).setInt32(this.len, v, true); this.len += 4; }
  f32(v: number) { this.ensure(4); new DataView(this.buf.buffer).setFloat32(this.len, v, true); this.len += 4; }
  f64(v: number) { this.ensure(8); new DataView(this.buf.buffer).setFloat64(this.len, v, true); this.len += 8; }
  zeros(n: number) { this.ensure(n); this.len += n; }

  setU32(offset: number, v: number) { new DataView(this.buf.buffer).setUint32(offset, v >>> 0, true); }
  setU16(offset: number, v: number) { new DataView(this.buf.buffer).setUint16(offset, v, true); }

  get position() { return this.len; }
  bytes(): Uint8Array { return this.buf.slice(0, this.len); }
}

// ─── CRC32 (IEEE 802.3, skips checksum field at bytes 24..27) ────────────────

function crc32Byte(byte: number): number {
  let crc = byte;
  for (let i = 0; i < 8; i++) {
    crc = crc & 1 ? (crc >>> 1) ^ 0xedb88320 : crc >>> 1;
  }
  return crc >>> 0;
}

function computeFileCrc32(data: Uint8Array): number {
  let crc = 0xffffffff;
  for (let i = 0; i < data.length; i++) {
    if (i >= 24 && i < 28) continue; // skip checksum field
    crc = (crc32Byte((crc ^ data[i]) & 0xff) ^ (crc >>> 8)) >>> 0;
  }
  return (crc ^ 0xffffffff) >>> 0;
}

// ─── Constants (must match namb_format.h) ────────────────────────────────────

const MAGIC = 0x4e414d42; // "NAMB"
const FORMAT_VERSION = 1;

const ARCH_IDS: Record<string, number> = {
  Linear: 0,
  ConvNet: 1,
  LSTM: 2,
  WaveNet: 3,
};

// Matches NAM/activations.h ActivationType enum order
const ACTIVATION_TYPES: Record<string, number> = {
  Tanh: 0,
  Hardtanh: 1,
  Fasttanh: 2,
  ReLU: 3,
  LeakyReLU: 4,
  PReLU: 5,
  Sigmoid: 6,
  SiLU: 7,
  Hardswish: 8,
  LeakyHardtanh: 9,
  LeakyHardTanh: 9, // both casings accepted, as in activations.cpp
  Softsign: 10,
};

const GATING_NONE = 0;
const GATING_GATED = 1;
const GATING_BLENDED = 2;

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Json = any;

// ─── Activation config ────────────────────────────────────────────────────────

function activationTypeId(name: string): number {
  const id = ACTIVATION_TYPES[name];
  if (id === undefined) throw new Error(`Unknown activation type: ${name}`);
  return id;
}

function writeActivationConfig(w: BinaryWriter, act: Json) {
  let type: number;
  const params: number[] = [];

  if (typeof act === 'string') {
    type = activationTypeId(act);
  } else if (act && typeof act === 'object') {
    type = activationTypeId(act.type);
    const typeName = Object.keys(ACTIVATION_TYPES).find((k) => ACTIVATION_TYPES[k] === type);
    if (typeName === 'LeakyReLU') {
      params.push(act.negative_slope ?? 0.01);
    } else if (typeName === 'PReLU') {
      if (act.negative_slope !== undefined) params.push(act.negative_slope);
      else if (Array.isArray(act.negative_slopes)) params.push(...act.negative_slopes);
    } else if (typeName === 'LeakyHardtanh') {
      params.push(act.min_val ?? -1.0, act.max_val ?? 1.0, act.min_slope ?? 0.01, act.max_slope ?? 0.01);
    }
  } else {
    throw new Error('Invalid activation config: expected string or object');
  }

  // LeakyHardtanh always carries its 4 params, even from string form
  if (typeof act === 'string' && (act === 'LeakyHardtanh' || act === 'LeakyHardTanh')) {
    params.push(-1.0, 1.0, 0.01, 0.01);
  }

  if (params.length > 255) throw new Error('Activation has too many parameters (max 255)');
  w.u8(type);
  w.u8(params.length);
  for (const p of params) w.f32(p);
}

// ─── FiLM params (4 bytes) ────────────────────────────────────────────────────

function writeFilmParams(w: BinaryWriter, layer: Json, key: string) {
  const film = layer[key];
  if (film === undefined || film === false) {
    w.u8(0); // flags: not active
    w.u8(0); // reserved
    w.u16(1); // groups (default)
    return;
  }
  const active = film.active ?? true;
  const shift = film.shift ?? true;
  const groups = film.groups ?? 1;
  let flags = 0;
  if (active) flags |= 0x01;
  if (shift) flags |= 0x02;
  w.u8(flags);
  w.u8(0);
  w.u16(groups);
}

// ─── Gating ───────────────────────────────────────────────────────────────────

function gatingModeFromString(s: string): number {
  if (s === 'gated') return GATING_GATED;
  if (s === 'blended') return GATING_BLENDED;
  if (s === 'none') return GATING_NONE;
  throw new Error(`Invalid gating_mode: ${s}`);
}

function parseGatingModes(layer: Json, numDilations: number): number[] {
  if (layer.gating_mode !== undefined) {
    if (Array.isArray(layer.gating_mode)) {
      return layer.gating_mode.map((gm: string) => gatingModeFromString(gm));
    }
    return new Array(numDilations).fill(gatingModeFromString(layer.gating_mode));
  }
  if (layer.gated !== undefined) {
    return new Array(numDilations).fill(layer.gated ? GATING_GATED : GATING_NONE);
  }
  return new Array(numDilations).fill(GATING_NONE);
}

// ─── Metadata block (48 bytes) ────────────────────────────────────────────────

const META_HAS_LOUDNESS = 0x01;
const META_HAS_INPUT_LEVEL = 0x02;
const META_HAS_OUTPUT_LEVEL = 0x04;

function writeMetadataBlock(w: BinaryWriter, model: Json) {
  const versionStr: string = model.version ?? '0.0.0';
  const [major = 0, minor = 0, patch = 0] = versionStr.split('.').map((s: string) => parseInt(s, 10) || 0);
  w.u8(major);
  w.u8(minor);
  w.u8(patch);

  let metaFlags = 0;
  let loudness = 0, inputLevel = 0, outputLevel = 0;
  const meta = model.metadata;
  if (meta && typeof meta === 'object') {
    if (meta.loudness !== undefined && meta.loudness !== null) {
      metaFlags |= META_HAS_LOUDNESS;
      loudness = meta.loudness;
    }
    if (meta.input_level_dbu !== undefined && meta.input_level_dbu !== null) {
      metaFlags |= META_HAS_INPUT_LEVEL;
      inputLevel = meta.input_level_dbu;
    }
    if (meta.output_level_dbu !== undefined && meta.output_level_dbu !== null) {
      metaFlags |= META_HAS_OUTPUT_LEVEL;
      outputLevel = meta.output_level_dbu;
    }
  }
  w.u8(metaFlags);

  w.f64(model.sample_rate !== undefined ? model.sample_rate : -1.0);
  w.f64(loudness);
  w.f64(inputLevel);
  w.f64(outputLevel);
  w.zeros(12); // reserved
}

// ─── Weight collection (condition_dsp weights first, recursively) ─────────────

function collectWeights(model: Json, out: number[]) {
  if (model.architecture === 'WaveNet' && model.config?.condition_dsp) {
    collectWeights(model.config.condition_dsp, out);
  }
  if (Array.isArray(model.weights)) {
    for (const w of model.weights) out.push(w);
  }
}

// ─── Per-architecture config blocks ──────────────────────────────────────────

function writeLinearConfig(w: BinaryWriter, config: Json) {
  w.i32(config.receptive_field);
  w.u8(config.bias ? 1 : 0);
  w.u8(config.in_channels ?? 1);
  w.u8(config.out_channels ?? 1);
  w.u8(0); // reserved
}

function writeLstmConfig(w: BinaryWriter, config: Json) {
  w.u16(config.num_layers);
  w.u16(config.input_size);
  w.u16(config.hidden_size);
  w.u8(config.in_channels ?? 1);
  w.u8(config.out_channels ?? 1);
  w.u16(0); // reserved
}

function writeConvnetConfig(w: BinaryWriter, config: Json) {
  const dilations: number[] = config.dilations;
  w.u16(config.channels);
  w.u8(config.batchnorm ? 1 : 0);
  w.u8(dilations.length);
  w.u16(config.groups ?? 1);
  w.u8(config.in_channels ?? 1);
  w.u8(config.out_channels ?? 1);
  writeActivationConfig(w, config.activation);
  for (const d of dilations) w.i32(d);
}

function writeWavenetConfig(w: BinaryWriter, model: Json) {
  const config = model.config;
  const inChannels = config.in_channels ?? 1;
  const withHead = config.head !== undefined && config.head !== null;
  const layerArrays: Json[] = config.layers;
  const hasConditionDsp = config.condition_dsp !== undefined;

  w.u8(inChannels);
  w.u8(withHead ? 1 : 0);
  w.u8(layerArrays.length);
  w.u8(hasConditionDsp ? 1 : 0);

  if (hasConditionDsp) {
    const cdsp = config.condition_dsp;
    const cdspWeights: number[] = [];
    collectWeights(cdsp, cdspWeights);
    w.u32(cdspWeights.length);
    writeMetadataBlock(w, cdsp);
    writeModelBlock(w, cdsp);
  }

  for (const layer of layerArrays) {
    const layerChannels: number = layer.channels;
    const bottleneck: number = layer.bottleneck ?? layerChannels;
    const dilations: number[] = layer.dilations;
    const numDilations = dilations.length;

    let kernelSizes: number[];
    if (Array.isArray(layer.kernel_sizes)) {
      kernelSizes = layer.kernel_sizes;
    } else if (layer.kernel_size !== undefined) {
      kernelSizes = new Array(numDilations).fill(layer.kernel_size);
    } else {
      throw new Error('Layer array missing kernel_size or kernel_sizes');
    }
    if (kernelSizes.length !== numDilations) {
      throw new Error('kernel_sizes length does not match dilations length');
    }

    let headSize = 0, headKernelSize = 1, headBias = false;
    if (layer.head !== undefined && layer.head !== null) {
      headSize = layer.head.out_channels;
      headKernelSize = layer.head.kernel_size;
      headBias = layer.head.bias;
    } else if (layer.head_size !== undefined) {
      headSize = layer.head_size;
      headBias = layer.head_bias;
    } else {
      throw new Error("Layer array missing 'head' or 'head_size'/'head_bias'");
    }

    w.u16(layer.input_size);
    w.u16(layer.condition_size);
    w.u16(headSize);
    w.u16(layerChannels);
    w.u16(bottleneck);
    w.u16(headKernelSize);

    w.u8(headBias ? 1 : 0);
    w.u8(numDilations);

    w.u16(layer.groups_input ?? 1);
    w.u16(layer.groups_input_mixin ?? 1);

    // layer1x1 params (4 bytes)
    let layer1x1Active = true, layer1x1Groups = 1;
    if (layer.layer1x1 !== undefined) {
      layer1x1Active = layer.layer1x1.active;
      layer1x1Groups = layer.layer1x1.groups;
    }
    w.u8(layer1x1Active ? 1 : 0);
    w.u16(layer1x1Groups);
    w.u8(0); // reserved

    // head1x1 params (6 bytes)
    let head1x1Active = false, head1x1OutChannels = layerChannels, head1x1Groups = 1;
    if (layer.head1x1 !== undefined) {
      head1x1Active = layer.head1x1.active;
      head1x1OutChannels = layer.head1x1.out_channels;
      head1x1Groups = layer.head1x1.groups;
    }
    w.u8(head1x1Active ? 1 : 0);
    w.u16(head1x1OutChannels);
    w.u16(head1x1Groups);
    w.u8(0); // reserved

    // 8 FiLM params (32 bytes)
    for (const key of [
      'conv_pre_film', 'conv_post_film',
      'input_mixin_pre_film', 'input_mixin_post_film',
      'activation_pre_film', 'activation_post_film',
      'layer1x1_post_film', 'head1x1_post_film',
    ]) {
      writeFilmParams(w, layer, key);
    }

    for (const d of dilations) w.i32(d);
    for (const ks of kernelSizes) w.u16(ks);

    // Activation configs
    if (Array.isArray(layer.activation)) {
      for (const act of layer.activation) writeActivationConfig(w, act);
    } else {
      for (let i = 0; i < numDilations; i++) writeActivationConfig(w, layer.activation);
    }

    // Gating modes
    const gatingModes = parseGatingModes(layer, numDilations);
    for (const gm of gatingModes) w.u8(gm);

    // Secondary activation configs
    for (let i = 0; i < numDilations; i++) {
      if (gatingModes[i] !== GATING_NONE) {
        if (layer.secondary_activation !== undefined) {
          writeActivationConfig(
            w,
            Array.isArray(layer.secondary_activation) ? layer.secondary_activation[i] : layer.secondary_activation,
          );
        } else {
          w.u8(ACTIVATION_TYPES.Sigmoid); // default secondary
          w.u8(0);
        }
      } else {
        w.u8(ACTIVATION_TYPES.Tanh); // type is ignored for NONE mode
        w.u8(0);
      }
    }
  }
}

function writeModelBlock(w: BinaryWriter, model: Json) {
  const archName: string = model.architecture;
  const arch = ARCH_IDS[archName];
  if (arch === undefined) throw new Error(`Unknown architecture: ${archName}`);

  w.u8(arch);
  w.u8(0); // reserved

  const configSizeOffset = w.position;
  w.u16(0); // placeholder, backpatched below
  const configStart = w.position;

  switch (arch) {
    case 0: writeLinearConfig(w, model.config); break;
    case 1: writeConvnetConfig(w, model.config); break;
    case 2: writeLstmConfig(w, model.config); break;
    case 3: writeWavenetConfig(w, model); break;
  }

  const configSize = w.position - configStart;
  if (configSize > 65535) throw new Error('Config too large for uint16');
  w.setU16(configSizeOffset, configSize);
}

// ─── SlimmableContainer resolution ───────────────────────────────────────────

// Mirrors ContainerModel::SetSlimmableSize: selects the first submodel whose
// max_value > slim_factor, or the last if slim_factor reaches the end.
export function resolveSlimmableContainer(model: Json, slimFactor = 0.0): Json {
  const submodels: Json[] = model.config?.submodels;
  if (!Array.isArray(submodels) || submodels.length === 0) {
    throw new Error("SlimmableContainer: 'submodels' must be a non-empty array");
  }
  let activeIndex = submodels.length - 1;
  for (let i = 0; i < submodels.length; i++) {
    if (slimFactor < submodels[i].max_value) {
      activeIndex = i;
      break;
    }
  }
  const selected = { ...submodels[activeIndex].model };
  for (const key of ['version', 'metadata', 'sample_rate']) {
    if (model[key] !== undefined && selected[key] === undefined) selected[key] = model[key];
  }
  return selected;
}

// ─── Main conversion ─────────────────────────────────────────────────────────

export interface NambResult {
  data: Uint8Array;
  numWeights: number;
  architecture: string;
  sampleRate: number | null;
}

// Weight count the NAMPedal firmware accepts (A2-nano WaveNet).
export const PEDAL_WEIGHT_COUNT = 1871;

export function convertNamToNamb(namJson: Json, slimFactor = 0.0): NambResult {
  let model = namJson;
  if (model.architecture === 'SlimmableContainer') {
    model = resolveSlimmableContainer(model, slimFactor);
  }

  const w = new BinaryWriter();

  // ---- File header (32 bytes) ----
  w.u32(MAGIC);
  w.u16(FORMAT_VERSION);
  w.u16(0); // flags
  const totalFileSizeOffset = w.position;
  w.u32(0);
  const weightsOffsetPos = w.position;
  w.u32(0);
  const totalWeightCountOffset = w.position;
  w.u32(0);
  const modelBlockSizeOffset = w.position;
  w.u32(0);
  w.u32(0); // checksum (backpatched)
  w.u32(0); // reserved

  // ---- Metadata block (48 bytes at offset 32) ----
  writeMetadataBlock(w, model);

  // ---- Model block (variable, at offset 80) ----
  const modelBlockStart = w.position;
  writeModelBlock(w, model);
  const modelBlockSize = w.position - modelBlockStart;

  // ---- Padding to align weights to 4 bytes ----
  while (w.position % 4 !== 0) w.u8(0);
  const weightsOffset = w.position;

  // ---- Weight data ----
  const allWeights: number[] = [];
  collectWeights(model, allWeights);
  for (const wt of allWeights) w.f32(wt);

  // ---- Backpatch header ----
  w.setU32(totalFileSizeOffset, w.position);
  w.setU32(weightsOffsetPos, weightsOffset);
  w.setU32(totalWeightCountOffset, allWeights.length);
  w.setU32(modelBlockSizeOffset, modelBlockSize);

  const data = w.bytes();
  const checksum = computeFileCrc32(data);
  new DataView(data.buffer, data.byteOffset).setUint32(24, checksum, true);

  return {
    data,
    numWeights: allWeights.length,
    architecture: model.architecture,
    sampleRate: model.sample_rate ?? null,
  };
}
