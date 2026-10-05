// Byte-compares the TypeScript bank builder against pack_models.py output.
// Usage: node scripts/verify-bank.ts <ref_bank.bin> NAME:file.namb [NAME:file.namb ...]
import { readFileSync } from 'node:fs';
import { buildBank } from '../src/lib/bank.ts';
import type { BankModel } from '../src/lib/bank.ts';

const [refPath, ...specs] = process.argv.slice(2);
const ref = readFileSync(refPath);

const models: BankModel[] = specs.map((spec) => {
  const idx = spec.indexOf(':');
  return { name: spec.slice(0, idx), namb: new Uint8Array(readFileSync(spec.slice(idx + 1))), ir: null };
});

const out = buildBank(models);
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
console.log(`identical (${out.length} bytes, ${models.length} models)`);
