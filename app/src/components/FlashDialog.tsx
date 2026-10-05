// Build bank -> DFU trigger -> WebUSB flash.
import { useEffect, useState } from 'react';
import { X } from 'lucide-react';
import type { Preset } from '../lib/flash';
import { buildBankImage, flashBank, isReady } from '../lib/flash';
import { triggerDfuOverSerial, isWebSerialSupported } from '../lib/serial';
import { isWebUsbSupported } from '../lib/webdfu';
import { t3kClient } from '../client';
import { Spinner } from './Spinner';

type Phase =
  | { step: 'building' }
  | { step: 'ready'; bank: Uint8Array; triggering: boolean }
  | { step: 'flashing'; percent: number; phase: 'erase' | 'write' }
  | { step: 'done'; bytes: number }
  | { step: 'error'; message: string; bank: Uint8Array | null };

async function fetchFile(modelUrl: string): Promise<ArrayBuffer> {
  const res = await t3kClient.fetchFile(modelUrl);
  if (!res.ok) throw new Error(`Download failed: ${res.status}`);
  return res.arrayBuffer();
}

const isDismiss = (err: unknown) => (err as DOMException)?.name === 'NotFoundError';
const message = (err: unknown) => String((err as Error)?.message ?? err);

interface Props {
  presets: Preset[];
  onClose: () => void;
}

export function FlashDialog({ presets, onClose }: Props) {
  const [phase, setPhase] = useState<Phase>({ step: 'building' });

  useEffect(() => {
    let canceled = false;
    buildBankImage(presets, fetchFile)
      .then((bank) => !canceled && setPhase({ step: 'ready', bank, triggering: false }))
      .catch((err) => !canceled && setPhase({ step: 'error', message: message(err), bank: null }));
    return () => {
      canceled = true;
    };
  }, [presets]);

  const trigger = async (bank: Uint8Array) => {
    setPhase({ step: 'ready', bank, triggering: true });
    try {
      await triggerDfuOverSerial();
      await new Promise((r) => setTimeout(r, 4000)); // re-enumerate as DFU
      setPhase({ step: 'ready', bank, triggering: false });
    } catch (err) {
      if (isDismiss(err)) setPhase({ step: 'ready', bank, triggering: false });
      else setPhase({ step: 'error', message: `DFU trigger failed: ${message(err)}`, bank });
    }
  };

  const flash = async (bank: Uint8Array) => {
    setPhase({ step: 'flashing', percent: 0, phase: 'erase' });
    try {
      await flashBank(bank, (written, total, p) =>
        setPhase({ step: 'flashing', percent: Math.round((written / total) * 100), phase: p }),
      );
      setPhase({ step: 'done', bytes: bank.length });
    } catch (err) {
      if (isDismiss(err)) setPhase({ step: 'ready', bank, triggering: false });
      else setPhase({ step: 'error', message: message(err), bank });
    }
  };

  const busy = phase.step === 'flashing';
  const filled = presets.filter(isReady).length;

  return (
    <div className="backdrop" onClick={busy ? undefined : onClose}>
      <div className="dialog" onClick={(e) => e.stopPropagation()}>
        <span className="label">Flash to pedal</span>
        {!busy && phase.step !== 'done' && (
          <button className="btn-icon plain dialog-close" onClick={onClose} aria-label="Close">
            <X size={14} />
          </button>
        )}

        {phase.step === 'building' && (
          <div className="status-row">
            <Spinner />
            <span className="muted">Building bank image</span>
          </div>
        )}

        {phase.step === 'ready' && (
          <>
            <p className="muted">
              {filled} of {presets.length} presets, {(phase.bank.length / 1024).toFixed(1)} KB.
              {!isWebUsbSupported() && ' WebUSB is not supported in this browser; use Chrome or Edge.'}
            </p>
            <div className="step">
              <span className="step-num">1</span>
              <div className="step-body">
                <strong>Put the pedal in DFU mode</strong>
                <p>Send the trigger over USB serial. The LEDs blink twice and the pedal resets. Or hold BOOT and tap RESET on the Seed.</p>
                <button className="btn" disabled={!isWebSerialSupported() || phase.triggering} onClick={() => trigger(phase.bank)}>
                  {phase.triggering ? 'Waiting for pedal' : 'Send DFU trigger'}
                </button>
              </div>
            </div>
            <div className="step">
              <span className="step-num">2</span>
              <div className="step-body">
                <strong>Connect and flash</strong>
                <p>Pick the DFU in FS Mode device in the browser prompt.</p>
                <button className="btn btn-primary" disabled={!isWebUsbSupported() || phase.triggering} onClick={() => flash(phase.bank)}>
                  Connect and flash
                </button>
              </div>
            </div>
          </>
        )}

        {phase.step === 'flashing' && (
          <div className="status">
            <span className="muted">
              {phase.phase === 'erase' ? 'Erasing' : 'Writing'} {phase.percent}%
            </span>
            <div className="progress">
              <div className="progress-bar" style={{ width: `${phase.percent}%` }} />
            </div>
            <span className="muted">Do not unplug the pedal.</span>
          </div>
        )}

        {phase.step === 'done' && (
          <div className="status">
            <p>
              Flashed {(phase.bytes / 1024).toFixed(1)} KB. The pedal is rebooting and will load the preset the
              rotary switch points at.
            </p>
            <button className="btn btn-primary" onClick={onClose}>Done</button>
          </div>
        )}

        {phase.step === 'error' && (
          <div className="status">
            <p className="error-text">{phase.message}</p>
            <div className="status-row">
              {phase.bank && (
                <button className="btn" onClick={() => setPhase({ step: 'ready', bank: phase.bank!, triggering: false })}>
                  Try again
                </button>
              )}
              <button className="btn" onClick={onClose}>Close</button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
