// Web Serial link to the running firmware's USB CDC port. One-byte commands:
//   'D'  reset into the Daisy bootloader (DFU) for flashing
//   'B'  print the preset bank directory (see readBankDirectory)

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

export const isWebSerialSupported = () => typeof navigator !== 'undefined' && 'serial' in navigator;

async function openPedalPort(): Promise<SerialPort> {
  const port = await navigator.serial.requestPort({ filters: [{ usbVendorId: 0x0483 }] });
  await port.open({ baudRate: 115200 });
  await port.setSignals({ dataTerminalReady: true }); // macOS CDC needs DTR
  await sleep(200);
  return port;
}

export async function triggerDfuOverSerial(): Promise<void> {
  const port = await openPedalPort();
  try {
    const writer = port.writable!.getWriter();
    await writer.write(new Uint8Array([0x44])); // 'D'
    writer.releaseLock();
    await sleep(500);
  } finally {
    await port.close().catch(() => undefined);
  }
}

// ─── Bank directory ──────────────────────────────────────────────────────────
//
// The firmware answers 'B' with, between fixed markers:
//   version=2 presets=4 current=2
//   preset=1 nam=1 nam_id=476289 block2=ir block2_id=1234 name=Plexi
//   ...
// or `error=...`. The name is the last field and runs to end of line.

export type Block2Kind = 'nam' | 'ir' | 'none';

export interface PedalPresetEntry {
  name: string;
  hasNam: boolean;
  namModelId: number; // 0 = unknown (older bank, or packed from local files)
  block2: Block2Kind;
  block2ModelId: number;
}

export interface PedalBank {
  current: number; // 0-based index of the preset the rotary points at
  presets: PedalPresetEntry[];
}

const BEGIN = 'T3K-BANK-BEGIN';
const END = 'T3K-BANK-END';

export function parseBankDirectory(text: string): PedalBank {
  const start = text.indexOf(BEGIN);
  const end = text.indexOf(END, start + BEGIN.length);
  if (start < 0 || end < 0) throw new Error("No bank directory in the pedal's reply.");
  const lines = text
    .slice(start + BEGIN.length, end)
    .split(/\r?\n/)
    .map((l) => l.trim())
    .filter(Boolean);

  let current = 0;
  const presets: PedalPresetEntry[] = [];
  for (const line of lines) {
    if (line.startsWith('error=')) throw new Error(`Pedal reports: ${line.slice(6)}`);
    const nameAt = line.indexOf(' name=');
    const head = nameAt >= 0 ? line.slice(0, nameAt) : line;
    const fields = new Map<string, string>();
    for (const kv of head.split(/\s+/)) {
      const eq = kv.indexOf('=');
      if (eq > 0) fields.set(kv.slice(0, eq), kv.slice(eq + 1));
    }
    if (fields.has('presets')) {
      current = Math.max(0, (parseInt(fields.get('current') ?? '1', 10) || 1) - 1);
      continue;
    }
    if (!fields.has('preset')) continue;
    const block2 = fields.get('block2');
    presets.push({
      name: nameAt >= 0 ? line.slice(nameAt + 6) : '',
      hasNam: fields.get('nam') === '1',
      namModelId: parseInt(fields.get('nam_id') ?? '0', 10) || 0,
      block2: block2 === 'nam' || block2 === 'ir' ? block2 : 'none',
      block2ModelId: parseInt(fields.get('block2_id') ?? '0', 10) || 0,
    });
  }
  return { current, presets };
}

// Ask the plugged-in pedal for its bank directory. Times out when the firmware
// doesn't answer (older firmware without the 'B' command, or not the pedal).
export async function readBankDirectory(timeoutMs = 3000): Promise<PedalBank> {
  const port = await openPedalPort();
  const decoder = new TextDecoder();
  let text = '';
  try {
    const writer = port.writable!.getWriter();
    await writer.write(new Uint8Array([0x42])); // 'B'
    writer.releaseLock();

    const reader = port.readable!.getReader();
    const deadline = Date.now() + timeoutMs;
    try {
      while (!text.includes(END)) {
        const remaining = deadline - Date.now();
        if (remaining <= 0) break;
        const timer = new Promise<'timeout'>((r) => setTimeout(() => r('timeout'), remaining));
        const result = await Promise.race([reader.read(), timer]);
        if (result === 'timeout') {
          await reader.cancel().catch(() => undefined);
          break;
        }
        if (result.done) break;
        text += decoder.decode(result.value, { stream: true });
      }
    } finally {
      reader.releaseLock();
    }
  } finally {
    await port.close().catch(() => undefined);
  }
  if (!text.includes(END)) {
    throw new Error('The pedal did not answer. Update its firmware to a version with "Sync from pedal" support.');
  }
  return parseBankDirectory(text);
}
