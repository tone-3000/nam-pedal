// Preset slots -> .namb conversion -> bank image -> DfuSe write to QSPI.

import { convertNamToNamb, PEDAL_WEIGHT_COUNT } from './namb';
import { buildBank, decodeWavChannel0At48k, processIr, MODEL_BANK_ADDR, MODEL_BANK_MAX_SIZE } from './bank';
import type { BankModel } from './bank';
import { DfuDevice } from './webdfu';
import type { FlashProgress } from './webdfu';
import type { Model, Tone } from '../types';

// A chain block: a tone with its selectable models and the active one.
export interface Block {
  tone: Tone;
  models: Model[];
  model: Model;
}

// NAM block carries the parsed .nam once validated; null while checking.
export interface NamBlock extends Block {
  namJson: unknown | null;
}

// Preset chain: NAM on block 1, optional cab IR on block 2.
export interface Preset {
  nam: NamBlock | null;
  ir: Block | null;
}
export type BlockKind = keyof Preset;

// Rotary position N loads bank entry N-1, so the bank always has NUM_PRESETS entries.
export const NUM_PRESETS = 4;
export const emptyPresets = (): Preset[] => Array.from({ length: NUM_PRESETS }, () => ({ nam: null, ir: null }));
export const isReady = (p: Preset) => p.nam?.namJson != null;

// Firmware treats a zero-size entry as "no model" and passes audio through.
const EMPTY_PRESET_NAME = '(empty)';

export const isNamFile = (m: Model) => m.model_url.toLowerCase().endsWith('.nam');
export const isIrFile = (m: Model) => m.model_url.toLowerCase().endsWith('.wav');

/** Parse and convert a .nam so incompatible models are rejected before staging. */
export function analyzeNamFile(text: string): { namJson: unknown; numWeights: number; architecture: string } {
  const namJson = JSON.parse(text);
  const { numWeights, architecture } = convertNamToNamb(namJson);
  return { namJson, numWeights, architecture };
}

export const isPedalCompatible = (numWeights: number) => numWeights === PEDAL_WEIGHT_COUNT;

export async function buildBankImage(
  presets: Preset[],
  fetchFile: (modelUrl: string) => Promise<ArrayBuffer>,
): Promise<Uint8Array> {
  if (!presets.some(isReady)) throw new Error('No presets assigned');

  const models: BankModel[] = [];
  for (const p of presets) {
    if (!isReady(p)) {
      models.push({ name: EMPTY_PRESET_NAME, namb: new Uint8Array(0), ir: null });
      continue;
    }
    const namb = convertNamToNamb(p.nam!.namJson).data;
    const ir = p.ir ? processIr(await decodeWavChannel0At48k(await fetchFile(p.ir.model.model_url)), 48000) : null;
    models.push({ name: p.nam!.model.name, namb, ir });
  }

  const bank = buildBank(models);
  if (bank.length > MODEL_BANK_MAX_SIZE) {
    throw new Error(`Bank is ${(bank.length / 1024).toFixed(0)} KB, limit is ${MODEL_BANK_MAX_SIZE / 1024} KB`);
  }
  return bank;
}

/** Caller puts the pedal in DFU mode first. */
export async function flashBank(bank: Uint8Array, progress?: FlashProgress): Promise<void> {
  const dfu = await DfuDevice.request();
  try {
    await dfu.flash(MODEL_BANK_ADDR, bank, progress);
    await dfu.leave();
  } catch (err) {
    await dfu.close();
    throw err;
  }
}
