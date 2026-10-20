// Byte-compares the TypeScript .nam→.namb converter against reference output
// from the C++ nam2namb tool. Run with: node scripts/verify-namb.ts <namDir> <nambDir>
import { readFileSync, readdirSync } from 'node:fs';
import { join } from 'node:path';
import { convertNamToNamb } from '../src/lib/namb.ts';

const [namDir, nambDir] = process.argv.slice(2);
let failures = 0;

for (const file of readdirSync(namDir).filter((f) => f.endsWith('.nam'))) {
  const name = file.replace(/\.nam$/, '');
  let ref: Buffer;
  try {
    ref = readFileSync(join(nambDir, `${name}.namb`));
  } catch {
    console.log(`${name}: no reference, skipped`);
    continue;
  }
  const json = JSON.parse(readFileSync(join(namDir, file), 'utf8'));
  let out: Uint8Array;
  try {
    out = convertNamToNamb(json, 0.0).data;
  } catch (err) {
    console.log(`${name}: CONVERT FAILED: ${(err as Error).message}`);
    failures++;
    continue;
  }
  if (out.length !== ref.length) {
    console.log(`${name}: SIZE MISMATCH ts=${out.length} ref=${ref.length}`);
    failures++;
    continue;
  }
  let diff = -1;
  for (let i = 0; i < out.length; i++) {
    if (out[i] !== ref[i]) { diff = i; break; }
  }
  if (diff >= 0) {
    console.log(`${name}: BYTE MISMATCH at offset ${diff} (ts=0x${out[diff].toString(16)} ref=0x${ref[diff].toString(16)})`);
    failures++;
  } else {
    console.log(`${name}: identical (${out.length} bytes)`);
  }
}

process.exit(failures ? 1 : 0);
