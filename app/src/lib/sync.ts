// "Sync from pedal": turn the bank directory read over serial into preset
// state, resolving each TONE3000 model id back to its tone and sibling models
// (Get Model -> Get Tone -> List Models). Blocks that can't be resolved (id 0
// from an older bank or a command-line pack, a tone made private or deleted)
// are left empty and reported in `warnings`.

import type { Block, NamBlock, Preset } from './flash';
import { NUM_PRESETS, emptyPresets, isNamFile, isIrFile } from './flash';
import type { PedalBank } from './serial';
import type { Model, Tone, PaginatedResponse } from '../types';

export interface SyncClient {
  getModel(id: number): Promise<Model>;
  getTone(id: number): Promise<Tone>;
  listModels(toneId: number): Promise<PaginatedResponse<Model>>;
}

export interface SyncResult {
  presets: Preset[]; // NAM blocks have namJson = null; the caller validates them
  selected: number;
  warnings: string[];
}

async function resolveBlock(client: SyncClient, modelId: number, want: 'nam' | 'ir'): Promise<Block> {
  const model = await client.getModel(modelId);
  const [tone, list] = await Promise.all([client.getTone(model.tone_id), client.listModels(model.tone_id)]);
  const models = list.data.filter(want === 'nam' ? isNamFile : isIrFile);
  // The pedal's model is the active one; make sure it's in the list even if the
  // architecture filter dropped it.
  if (!models.some((m) => m.id === model.id)) models.unshift(model);
  return { tone, models, model };
}

export async function resolvePedalBank(bank: PedalBank, client: SyncClient): Promise<SyncResult> {
  const presets = emptyPresets();
  const warnings: string[] = [];

  for (let i = 0; i < Math.min(bank.presets.length, NUM_PRESETS); i++) {
    const entry = bank.presets[i];
    if (!entry.hasNam) continue;
    const warn = (what: string, why: string) => warnings.push(`Preset ${i + 1} (${entry.name || 'unnamed'}): ${what} ${why}`);

    let nam: NamBlock | null = null;
    if (!entry.namModelId) warn('block 1', 'is not linked to a TONE3000 model.');
    else {
      try {
        nam = { ...(await resolveBlock(client, entry.namModelId, 'nam')), namJson: null };
      } catch (err) {
        warn('block 1', `could not be loaded from TONE3000 (${(err as Error).message}).`);
      }
    }

    let nam2: NamBlock | null = null;
    let ir: Block | null = null;
    if (entry.block2 !== 'none') {
      if (!entry.block2ModelId) warn('block 2', 'is not linked to a TONE3000 model.');
      else {
        try {
          const block = await resolveBlock(client, entry.block2ModelId, entry.block2);
          if (entry.block2 === 'nam') nam2 = { ...block, namJson: null };
          else ir = block;
        } catch (err) {
          warn('block 2', `could not be loaded from TONE3000 (${(err as Error).message}).`);
        }
      }
    }

    // Block 2 without block 1 is not a valid chain.
    if (nam) presets[i] = { nam, nam2, ir };
  }

  return { presets, selected: Math.min(bank.current, NUM_PRESETS - 1), warnings };
}
