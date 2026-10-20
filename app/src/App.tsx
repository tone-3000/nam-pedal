import { useState, useEffect, useRef } from 'react';
import { X } from 'lucide-react';
import { PUBLISHABLE_KEY, REDIRECT_URI } from './config';
import { startOAuthPopup, handleOAuthCallbackFromPopup } from './tone3000-client';
import type { OAuthPopupOptions } from './tone3000-client';
import { t3kClient } from './client';
import type { Model, EmbeddedUser } from './types';
import type { Preset, BlockKind, NamKind, Slot } from './lib/flash';
import { analyzeNamFile, isPedalCompatible, emptyPresets, isNamFile, isIrFile, isNamKind } from './lib/flash';
import { PEDAL_WEIGHT_COUNT } from './lib/namb';
import { readBankDirectory } from './lib/serial';
import { resolvePedalBank } from './lib/sync';
import { loadPresets, savePresets } from './lib/store';
import { Header } from './components/Header';
import { Splash } from './components/Splash';
import { Chain } from './components/Chain';
import { BlockDetail } from './components/BlockDetail';
import { Pedal } from './components/Pedal';
import { FlashDialog } from './components/FlashDialog';
import { Spinner } from './components/Spinner';

// Popup side of the OAuth flow: relay the callback to the opener and close.
// BroadcastChannel covers the case where window.opener was cleared by a cross-origin login.
(function relayPopupCallback() {
  if (!(window.opener || sessionStorage.getItem('t3k_popup_mode') === '1')) return;
  const q = new URLSearchParams(window.location.search);
  if (!q.has('code') && !(q.has('error') && q.has('state')) && !q.has('canceled')) return;
  const msg = {
    type: 't3k_oauth_callback',
    code: q.get('code'),
    state: q.get('state'),
    error: q.get('error'),
    tone_id: q.get('tone_id'),
    canceled: q.get('canceled') === 'true',
  };
  if (window.opener) {
    window.opener.postMessage(msg, window.location.origin);
  } else {
    const bc = new BroadcastChannel('t3k_oauth');
    bc.postMessage(msg);
    bc.close();
  }
  window.close();
})();

const KINDS = ['nam', 'nam2', 'ir'] as const;
const LOGIN_OPTIONS: OAuthPopupOptions = { menubar: true };
// Slot 1 is an A2 capture. Slot 2 takes a second capture or a cab IR, so its
// picker is open to both formats and every gear but spaces (room/reverb IRs);
// IR tones pass through the architecture filter untouched.
const SELECT_OPTIONS: Record<Slot, OAuthPopupOptions> = {
  1: { prompt: 'select_tone', format: 'nam', architecture: 2, menubar: true, preview: true },
  2: { prompt: 'select_tone', architecture: 2, gears: 'amp_amp-cab_pedal_outboard_cab_experimental', menubar: true, preview: true },
};

// The last session's presets come back from localStorage (without the parsed
// .nam, which is re-downloaded once signed in).
const restored = loadPresets();

export default function App() {
  const [user, setUser] = useState<EmbeddedUser | null>(null);
  const [entered, setEntered] = useState(false);
  const [presets, setPresets] = useState<Preset[]>(() => restored?.presets ?? emptyPresets());
  const [selected, setSelected] = useState(restored?.selected ?? 0);
  const [block, setBlock] = useState<BlockKind>('nam');
  const [loading, setLoading] = useState(false);
  // Number of .nam downloads in flight (restore and sync check several at once).
  const [checkCount, setCheckCount] = useState(0);
  const checking = checkCount > 0;
  const [browsing, setBrowsing] = useState(false);
  const [syncing, setSyncing] = useState(false);
  const [flashOpen, setFlashOpen] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);
  const popup = useRef<Window | null>(null);
  // Which chain slot the open select popup is for.
  const target = useRef<{ preset: number; slot: Slot }>({ preset: 0, slot: 1 });

  const updatePreset = (i: number, fn: (p: Preset) => Preset) =>
    setPresets((prev) => prev.map((p, j) => (j === i ? fn(p) : p)));

  // Remember the session for next time.
  useEffect(() => savePresets(presets, selected), [presets, selected]);

  // Download and validate a .nam; store the parsed JSON on the block or report why not.
  const checkNam = async (i: number, kind: NamKind, model: Model) => {
    setCheckCount((c) => c + 1);
    setError(null);
    try {
      const res = await t3kClient.fetchFile(model.model_url);
      if (!res.ok) throw new Error(`Download failed: ${res.status}`);
      const { namJson, numWeights, architecture } = analyzeNamFile(await res.text());
      if (!isPedalCompatible(numWeights)) {
        throw new Error(`${architecture} model with ${numWeights} weights. The pedal runs A2-Lite models only (${PEDAL_WEIGHT_COUNT} weights).`);
      }
      updatePreset(i, (p) => (p[kind]?.model.id === model.id ? { ...p, [kind]: { ...p[kind]!, namJson } } : p));
    } catch (err) {
      setError(`${model.name}: ${(err as Error).message}`);
    } finally {
      setCheckCount((c) => c - 1);
    }
  };

  // Re-validate every NAM block that has no parsed .nam yet (restored or synced).
  const checkPending = (list: Preset[]) => {
    list.forEach((p, i) => {
      for (const kind of ['nam', 'nam2'] as const) {
        const b = p[kind];
        if (b && b.namJson == null) checkNam(i, kind, b.model);
      }
    });
  };

  // Read the bank directory off the plugged-in pedal and rebuild the presets from it.
  const syncFromPedal = async () => {
    setError(null);
    setNotice(null);
    setSyncing(true);
    try {
      const bank = await readBankDirectory();
      const { presets: synced, selected: current, warnings } = await resolvePedalBank(bank, t3kClient);
      setPresets(synced);
      setSelected(current);
      setBlock('nam');
      checkPending(synced);
      if (warnings.length) setNotice(`Synced from pedal. ${warnings.join(' ')}`);
    } catch (err) {
      if ((err as DOMException)?.name !== 'NotFoundError') setError(`Sync failed: ${(err as Error).message}`);
    } finally {
      setSyncing(false);
    }
  };

  const setModel = (model: Model) => {
    const i = selected;
    if (isNamKind(block)) {
      updatePreset(i, (p) => (p[block] ? { ...p, [block]: { ...p[block]!, model, namJson: null } } : p));
      checkNam(i, block, model);
    } else {
      updatePreset(i, (p) => (p.ir ? { ...p, ir: { ...p.ir, model } } : p));
    }
  };

  // Restore a stored session.
  useEffect(() => {
    if (!t3kClient.isConnected()) return;
    t3kClient.getUser().then(setUser).catch(() => t3kClient.clearTokens());
  }, []);

  // Once in, fetch the .nam files for the presets restored from localStorage.
  useEffect(() => {
    if (entered) checkPending(presets);
  }, [entered]);

  useEffect(() => {
    const onMessage = async (event: MessageEvent) => {
      if (event.data?.type !== 't3k_oauth_callback') return;
      setBrowsing(false);
      const result = await handleOAuthCallbackFromPopup(PUBLISHABLE_KEY, REDIRECT_URI, event);
      if (!result) return;
      if (!result.ok) {
        if (result.error !== 'canceled') setError('Sign in failed. Try again.');
        return;
      }
      t3kClient.setTokens(result.tokens);
      setEntered(true);
      if (!user) t3kClient.getUser().then(setUser).catch(() => {});
      if (result.canceled || !result.toneId) return;

      const { preset: i, slot } = target.current;
      setLoading(true);
      try {
        const [tone, list] = await Promise.all([
          t3kClient.getTone(result.toneId),
          t3kClient.listModels(result.toneId),
        ]);
        // Slot 2 accepts either format; the tone's files decide which block it becomes.
        const nams = list.data.filter(isNamFile);
        const irs = slot === 2 ? list.data.filter(isIrFile) : [];
        const kind: BlockKind = nams.length ? (slot === 1 ? 'nam' : 'nam2') : 'ir';
        const models = kind === 'ir' ? irs : nams;
        if (models.length === 0) throw new Error(slot === 1 ? 'No A2 models in this tone.' : 'No A2 models or IR files in this tone.');
        const blk = { tone, models, model: models[0] };
        if (isNamKind(kind)) {
          // Slot 2 holds one block: a new NAM replaces any IR, and vice versa.
          updatePreset(i, (p) => ({ ...p, ir: kind === 'nam2' ? null : p.ir, [kind]: { ...blk, namJson: null } }));
          setLoading(false);
          await checkNam(i, kind, models[0]);
        } else {
          updatePreset(i, (p) => ({ ...p, nam2: null, ir: blk }));
        }
        setBlock(kind);
      } catch (err) {
        setError((err as Error).message || 'Could not load the tone. Try again.');
      } finally {
        setLoading(false);
      }
    };

    window.addEventListener('message', onMessage);
    const bc = new BroadcastChannel('t3k_oauth');
    bc.onmessage = onMessage;
    return () => {
      window.removeEventListener('message', onMessage);
      bc.close();
    };
  }, [user]);

  // Clear the browsing state if the popup is closed without a result.
  useEffect(() => {
    if (!browsing) return;
    const id = setInterval(() => {
      if (popup.current?.closed) {
        setBrowsing(false);
        popup.current = null;
      }
    }, 500);
    return () => clearInterval(id);
  }, [browsing]);

  const open = async (options: OAuthPopupOptions) => {
    setError(null);
    setBrowsing(true);
    popup.current = await startOAuthPopup(PUBLISHABLE_KEY, REDIRECT_URI, options);
  };
  const signedIn = !!user || t3kClient.isConnected();
  const enter = () => (signedIn ? setEntered(true) : open(LOGIN_OPTIONS));
  const select = (slot: Slot) => {
    target.current = { preset: selected, slot };
    open(SELECT_OPTIONS[slot]);
  };
  const remove = (kind: BlockKind) => updatePreset(selected, (p) => ({ ...p, [kind]: null }));

  const preset = presets[selected];
  // Show the chosen block, else fall back to whatever is assigned (block 1 first).
  const detailKind: BlockKind = (preset[block] ? block : KINDS.find((k) => preset[k])) ?? 'nam';
  const detail = preset[detailKind];

  return (
    <>
      <Header user={user} />

      {error && (
        <div className="banner banner-error banner-top">
          <span>{error}</span>
          <button className="btn-icon plain" onClick={() => setError(null)} aria-label="Dismiss">
            <X size={14} />
          </button>
        </div>
      )}
      {notice && (
        <div className="banner banner-top">
          <span>{notice}</span>
          <button className="btn-icon plain" onClick={() => setNotice(null)} aria-label="Dismiss">
            <X size={14} />
          </button>
        </div>
      )}

      {!entered && <Splash signedIn={signedIn} busy={browsing} onContinue={enter} />}

      {entered && (
        <main className="main">
          <section className="card">
            <div className="section-title">
              <span className="label">Preset {selected + 1}</span>
              {(loading || browsing) && <Spinner />}
            </div>
            <Chain
              preset={preset}
              active={detailKind}
              checking={checking}
              onOpen={setBlock}
              onSwap={select}
              onRemove={remove}
            />
            {detail && <BlockDetail kind={detailKind} block={detail} checking={checking} onModel={setModel} />}
          </section>
          <Pedal
            presets={presets}
            selected={selected}
            syncing={syncing}
            onSelect={(i) => {
              setSelected(i);
              setBlock('nam');
            }}
            onFlash={() => setFlashOpen(true)}
            onSync={syncFromPedal}
          />
        </main>
      )}

      {flashOpen && <FlashDialog presets={presets} onClose={() => setFlashOpen(false)} />}
    </>
  );
}
