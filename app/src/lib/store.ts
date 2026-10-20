// Preset state in localStorage, so the next visit from the same browser starts
// where the last one left off. The parsed .nam JSON is not stored (large, and
// re-downloaded on restore); everything else is.

import type { NamBlock, Preset } from './flash';
import { NUM_PRESETS, emptyPresets } from './flash';

const KEY = 't3k_pedal_presets_v1';

interface Stored {
  presets: Preset[];
  selected: number;
}

const stripNam = (b: NamBlock | null): NamBlock | null => (b ? { ...b, namJson: null } : null);

export function savePresets(presets: Preset[], selected: number): void {
  try {
    const stored: Stored = {
      presets: presets.map((p) => ({ ...p, nam: stripNam(p.nam), nam2: stripNam(p.nam2) })),
      selected,
    };
    localStorage.setItem(KEY, JSON.stringify(stored));
  } catch {
    // Storage full or disabled: the session still works, it just isn't remembered.
  }
}

export function loadPresets(): Stored | null {
  try {
    const raw = localStorage.getItem(KEY);
    if (!raw) return null;
    const stored = JSON.parse(raw) as Partial<Stored>;
    if (!Array.isArray(stored.presets)) return null;
    const presets = emptyPresets();
    stored.presets.slice(0, NUM_PRESETS).forEach((p, i) => {
      if (p && typeof p === 'object') {
        presets[i] = { nam: stripNam(p.nam ?? null), nam2: stripNam(p.nam2 ?? null), ir: p.ir ?? null };
      }
    });
    const selected = Number.isInteger(stored.selected) ? Math.min(Math.max(0, stored.selected!), NUM_PRESETS - 1) : 0;
    return { presets, selected };
  } catch {
    return null;
  }
}
