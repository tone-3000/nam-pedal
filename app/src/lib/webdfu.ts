// WebUSB DfuSe driver for the STM32 bootloader (port of pydfu.py).
// Implements enough of the DFU 1.1 + ST DfuSe extensions (AN3156) to erase
// and write the Daisy Seed's QSPI flash over USB.

const DFU_VID = 0x0483;
const DFU_PID = 0xdf11;

// DFU requests
const DFU_DNLOAD = 1;
const DFU_GETSTATUS = 3;
const DFU_CLRSTATUS = 4;
const DFU_ABORT = 6;

// DFU states
const STATE_DFU_IDLE = 0x02;
const STATE_DFU_DOWNLOAD_BUSY = 0x04;
const STATE_DFU_DOWNLOAD_IDLE = 0x05;
const STATE_DFU_MANIFEST = 0x07;
const STATE_DFU_UPLOAD_IDLE = 0x09;
const STATE_DFU_ERROR = 0x0a;

const STATE_NAMES: Record<number, string> = {
  0x00: 'APP_IDLE', 0x01: 'APP_DETACH', 0x02: 'DFU_IDLE',
  0x03: 'DOWNLOAD_SYNC', 0x04: 'DOWNLOAD_BUSY', 0x05: 'DOWNLOAD_IDLE',
  0x06: 'MANIFEST_SYNC', 0x07: 'MANIFEST', 0x08: 'MANIFEST_WAIT_RESET',
  0x09: 'UPLOAD_IDLE', 0x0a: 'ERROR',
};

export interface MemorySegment {
  addr: number;
  lastAddr: number;
  pageSize: number;
}

export type FlashProgress = (bytesWritten: number, totalBytes: number, phase: 'erase' | 'write') => void;

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

export class DfuDevice {
  private device: USBDevice;
  private interfaceNumber: number;
  private transferSize: number;
  private segments: MemorySegment[];

  private constructor(device: USBDevice, interfaceNumber: number, transferSize: number, segments: MemorySegment[]) {
    this.device = device;
    this.interfaceNumber = interfaceNumber;
    this.transferSize = transferSize;
    this.segments = segments;
  }

  /** Prompt the user to pick the DFU device and prepare it for programming. */
  static async request(): Promise<DfuDevice> {
    const device = await navigator.usb.requestDevice({
      filters: [{ vendorId: DFU_VID, productId: DFU_PID }],
    });
    await device.open();
    if (device.configuration === null) await device.selectConfiguration(1);

    // Find the DFU interface (class 0xFE, subclass 1)
    const intf = device.configuration!.interfaces.find((i) =>
      i.alternates.some((a) => a.interfaceClass === 0xfe && a.interfaceSubclass === 1),
    );
    if (!intf) throw new Error('No DFU interface found on device');
    await device.claimInterface(intf.interfaceNumber);
    // Alt 0 = main memory map (QSPI on the Daisy bootloader)
    await device.selectAlternateInterface(intf.interfaceNumber, 0);

    // The DfuSe memory layout lives in the alt-interface name string. Chrome
    // often doesn't populate interfaceName, so fall back to reading the raw
    // config descriptor + string descriptor from the device (like dfu-util).
    const config = await readConfigDescriptor(device);
    const alt0 = intf.alternates.find((a) => a.alternateSetting === 0) ?? intf.alternates[0];
    let layoutName = alt0.interfaceName ?? '';
    let segments = parseMemoryLayout(layoutName);
    if (segments.length === 0 && config) {
      const iInterface = findInterfaceStringIndex(config, intf.interfaceNumber, 0);
      if (iInterface > 0) {
        layoutName = await readStringDescriptor(device, iInterface);
        segments = parseMemoryLayout(layoutName);
      }
    }
    if (segments.length === 0) {
      throw new Error(
        'Could not read the device\'s flash memory layout' +
        (layoutName ? ` (got "${layoutName}")` : '') +
        '. Try unplugging and reconnecting the pedal.',
      );
    }

    const transferSize = (config && parseDfuTransferSize(config)) || 1024;
    const dfu = new DfuDevice(device, intf.interfaceNumber, transferSize, segments);
    await dfu.enterIdle();
    return dfu;
  }

  get memorySegments(): MemorySegment[] {
    return this.segments;
  }

  // ── USB plumbing ────────────────────────────────────────────────────────────

  private async controlOut(request: number, value: number, data?: BufferSource): Promise<void> {
    const result = await this.device.controlTransferOut(
      { requestType: 'class', recipient: 'interface', request, value, index: this.interfaceNumber },
      data,
    );
    if (result.status !== 'ok') throw new Error(`DFU control transfer failed: ${result.status}`);
  }

  private async controlIn(request: number, length: number): Promise<DataView> {
    const result = await this.device.controlTransferIn(
      { requestType: 'class', recipient: 'interface', request, value: 0, index: this.interfaceNumber },
      length,
    );
    if (result.status !== 'ok' || !result.data) {
      throw new Error(`DFU control transfer failed: ${result.status}`);
    }
    return result.data;
  }

  // ── DFU protocol ────────────────────────────────────────────────────────────

  private async getStatus(): Promise<{ state: number; pollTimeout: number }> {
    const d = await this.controlIn(DFU_GETSTATUS, 6);
    const pollTimeout = d.getUint8(1) | (d.getUint8(2) << 8) | (d.getUint8(3) << 16);
    return { state: d.getUint8(4), pollTimeout };
  }

  private async enterIdle(): Promise<void> {
    for (let attempt = 0; attempt < 4; attempt++) {
      const { state } = await this.getStatus();
      if (state === STATE_DFU_IDLE) return;
      if (state === STATE_DFU_DOWNLOAD_IDLE || state === STATE_DFU_UPLOAD_IDLE) {
        await this.controlOut(DFU_ABORT, 0);
      } else {
        await this.controlOut(DFU_CLRSTATUS, 0);
      }
    }
  }

  // Issue a DNLOAD command and poll status until the device settles.
  private async dnloadAndPoll(value: number, data: BufferSource | undefined, stage: string): Promise<void> {
    await this.controlOut(DFU_DNLOAD, value, data);
    // First status read triggers execution; poll while busy.
    for (;;) {
      const { state, pollTimeout } = await this.getStatus();
      if (state === STATE_DFU_DOWNLOAD_BUSY) {
        await sleep(pollTimeout);
        continue;
      }
      if (state === STATE_DFU_DOWNLOAD_IDLE || state === STATE_DFU_MANIFEST || state === STATE_DFU_IDLE) return;
      if (state === STATE_DFU_ERROR) {
        await this.controlOut(DFU_CLRSTATUS, 0);
        throw new Error(`DFU: ${stage} failed (device reported error)`);
      }
      throw new Error(`DFU: ${stage} failed (state=${STATE_NAMES[state] ?? state})`);
    }
  }

  private async pageErase(addr: number): Promise<void> {
    const buf = new Uint8Array(5);
    buf[0] = 0x41;
    new DataView(buf.buffer).setUint32(1, addr, true);
    await this.dnloadAndPoll(0, buf, `erase 0x${addr.toString(16)}`);
  }

  private async setAddress(addr: number): Promise<void> {
    const buf = new Uint8Array(5);
    buf[0] = 0x21;
    new DataView(buf.buffer).setUint32(1, addr, true);
    await this.dnloadAndPoll(0, buf, `set address 0x${addr.toString(16)}`);
  }

  private async writeMemory(addr: number, data: Uint8Array<ArrayBuffer>, onChunk?: (written: number) => void): Promise<void> {
    let written = 0;
    while (written < data.length) {
      await this.setAddress(addr + written);
      const chunk = data.subarray(written, written + Math.min(this.transferSize, data.length - written));
      // wBlockNum=2 → write to the address set above (DfuSe convention)
      await this.dnloadAndPoll(2, chunk, `write 0x${(addr + written).toString(16)}`);
      written += chunk.length;
      onChunk?.(written);
    }
  }

  /**
   * Erase the pages covering [addr, addr+data.length) and write data.
   * Mirrors pydfu.py write_elements().
   */
  async flash(addr: number, data: Uint8Array<ArrayBuffer>, progress?: FlashProgress): Promise<void> {
    if (this.segments.length === 0) {
      throw new Error('Device reported no memory layout; cannot erase');
    }
    const total = data.length;
    let offset = 0;

    while (offset < total) {
      const cur = addr + offset;
      const segment = this.segments.find((s) => cur >= s.addr && cur <= s.lastAddr);
      if (!segment) {
        throw new Error(
          `Address 0x${cur.toString(16)} is outside the device's writable memory. ` +
          `Is the Daisy bootloader (not the ROM bootloader) running?`,
        );
      }
      // No 32-bit bitwise ops: addresses like 0x90600000 exceed 2^31
      const pageAddr = Math.floor(cur / segment.pageSize) * segment.pageSize;
      const writeSize = Math.min(total - offset, pageAddr + segment.pageSize - cur);

      progress?.(offset, total, 'erase');
      await this.pageErase(pageAddr);

      progress?.(offset, total, 'write');
      const chunkStart = offset;
      await this.writeMemory(cur, data.subarray(offset, offset + writeSize), (w) => {
        progress?.(chunkStart + w, total, 'write');
      });

      offset += writeSize;
    }
    progress?.(total, total, 'write');
  }

  /** Exit DFU and run the application (mirrors pydfu.py exit_dfu()). */
  async leave(): Promise<void> {
    try {
      await this.setAddress(0x08000000);
      // Zero-length DNLOAD triggers manifest → device reboots into the app.
      await this.controlOut(DFU_DNLOAD, 0);
      await this.getStatus().catch(() => undefined); // device may drop off the bus here
    } catch {
      // The device resetting mid-request is expected
    }
    await this.close();
  }

  async close(): Promise<void> {
    try {
      await this.device.close();
    } catch {
      // already gone
    }
  }
}

// ─── Raw descriptor helpers ───────────────────────────────────────────────────
// WebUSB doesn't expose the DFU functional descriptor and often omits interface
// name strings, so they are read straight from the device.

const GET_DESCRIPTOR = 6;

async function readConfigDescriptor(device: USBDevice): Promise<DataView | null> {
  try {
    const result = await device.controlTransferIn(
      { requestType: 'standard', recipient: 'device', request: GET_DESCRIPTOR, value: 0x0200, index: 0 },
      4096,
    );
    return result.status === 'ok' && result.data ? result.data : null;
  } catch {
    return null;
  }
}

// Find iInterface (string descriptor index) for a given interface/alt setting
// by walking the raw configuration descriptor.
function findInterfaceStringIndex(config: DataView, interfaceNumber: number, altSetting: number): number {
  let pos = 0;
  while (pos + 2 <= config.byteLength) {
    const len = config.getUint8(pos);
    const type = config.getUint8(pos + 1);
    if (len === 0) break;
    // Interface descriptor: bLength=9, bDescriptorType=4
    if (type === 4 && len >= 9 && pos + 9 <= config.byteLength) {
      const num = config.getUint8(pos + 2);
      const alt = config.getUint8(pos + 3);
      if (num === interfaceNumber && alt === altSetting) {
        return config.getUint8(pos + 8); // iInterface
      }
    }
    pos += len;
  }
  return 0;
}

// Extract wTransferSize from the DFU functional descriptor (type 0x21).
function parseDfuTransferSize(config: DataView): number | null {
  let pos = 0;
  while (pos + 2 <= config.byteLength) {
    const len = config.getUint8(pos);
    const type = config.getUint8(pos + 1);
    if (len === 0) break;
    if (type === 0x21 && len === 9 && pos + 9 <= config.byteLength) {
      return config.getUint16(pos + 5, true);
    }
    pos += len;
  }
  return null;
}

async function readStringDescriptor(device: USBDevice, index: number): Promise<string> {
  try {
    const result = await device.controlTransferIn(
      {
        requestType: 'standard',
        recipient: 'device',
        request: GET_DESCRIPTOR,
        value: 0x0300 | index,
        index: 0x0409, // en-US
      },
      255,
    );
    if (result.status !== 'ok' || !result.data || result.data.byteLength < 2) return '';
    const d = result.data;
    const len = Math.min(d.getUint8(0), d.byteLength);
    let s = '';
    for (let i = 2; i + 1 < len; i += 2) {
      s += String.fromCharCode(d.getUint16(i, true));
    }
    return s;
  } catch {
    return '';
  }
}

// Parse a DfuSe memory layout string like
//   "@Internal Flash   /0x08000000/16*128Kg"  or
//   "@Flash /0x90000000/64*4Kg,..." (multi-segment)
export function parseMemoryLayout(name: string): MemorySegment[] {
  const segments: MemorySegment[] = [];
  const parts = name.split('/');
  for (let i = 1; i + 1 < parts.length; i += 2) {
    let addr = parseInt(parts[i], 16);
    for (const seg of parts[i + 1].split(',')) {
      const m = /(\d+)\*(\d+)(.)(.)?/.exec(seg.trim());
      if (!m) continue;
      const numPages = parseInt(m[1], 10);
      let pageSize = parseInt(m[2], 10);
      if (m[3] === 'K') pageSize *= 1024;
      if (m[3] === 'M') pageSize *= 1024 * 1024;
      const size = numPages * pageSize;
      segments.push({ addr, lastAddr: addr + size - 1, pageSize });
      addr += size;
    }
  }
  return segments;
}

export function isWebUsbSupported(): boolean {
  return typeof navigator !== 'undefined' && 'usb' in navigator;
}
