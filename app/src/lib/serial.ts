// Web Serial DFU trigger. The firmware resets into the Daisy bootloader on a 'D' byte.

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

export const isWebSerialSupported = () => typeof navigator !== 'undefined' && 'serial' in navigator;

export async function triggerDfuOverSerial(): Promise<void> {
  const port = await navigator.serial.requestPort({ filters: [{ usbVendorId: 0x0483 }] });
  await port.open({ baudRate: 115200 });
  try {
    await port.setSignals({ dataTerminalReady: true }); // macOS CDC needs DTR
    await sleep(200);
    const writer = port.writable!.getWriter();
    await writer.write(new Uint8Array([0x44])); // 'D'
    writer.releaseLock();
    await sleep(500);
  } finally {
    await port.close().catch(() => undefined);
  }
}
