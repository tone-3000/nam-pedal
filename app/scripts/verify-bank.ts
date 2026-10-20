// Byte-compares the TypeScript bank builder against pack_models.py output.
// Usage: node scripts/verify-bank.ts <ref_bank.bin> NAME:file.namb[:file2.namb] [...]
import { readFileSync } from 'node:fs';
import { buildBank } from '../src/lib/bank.ts';
import type { BankPreset } from '../src/lib/bank.ts';

const [refPath, ...specs] = process.argv.slice(2);
const ref = readFileSync(refPath);

const read = (path: string) => new Uint8Array(readFileSync(path));
const presets: BankPreset[] = specs.map((spec) => {
  const [name, namb, namb2] = spec.split(':');
  return { name, namb: read(namb), namb2: namb2 ? read(namb2) : null, ir: null };
});

const out = buildBank(presets);
if (out.length !== ref.length) {
  console.log(`SIZE MISMATCH ts=${out.length} ref=${ref.length}`);
  process.exit(1);
}
for (let i = 0; i < out.length; i++) {
  if (out[i] !== ref[i]) {
    console.log(`BYTE MISMATCH at offset ${i}`);
    process.exit(1);
  }
}
console.log(`identical (${out.length} bytes, ${presets.length} presets)`);
