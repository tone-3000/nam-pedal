import { useState, useEffect, useRef } from 'react';
import { X } from 'lucide-react';
import { PUBLISHABLE_KEY, REDIRECT_URI } from './config';
import { startOAuthPopup, handleOAuthCallbackFromPopup } from './tone3000-client';
import type { OAuthPopupOptions } from './tone3000-client';
import { t3kClient } from './client';
import type { Model, EmbeddedUser } from './types';
import type { Preset, BlockKind } from './lib/flash';
import { analyzeNamFile, isPedalCompatible, emptyPresets, isNamFile, isIrFile } from './lib/flash';
import { PEDAL_WEIGHT_COUNT } from './lib/namb';
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

const LOGIN_OPTIONS: OAuthPopupOptions = { menubar: true };
const SELECT_OPTIONS: Record<BlockKind, OAuthPopupOptions> = {
  nam: { prompt: 'select_tone', format: 'nam', architecture: 2, menubar: true, preview: true },
  ir: { prompt: 'select_tone', format: 'ir', gears: 'cab', menubar: true, preview: true },
};

export default function App() {
  const [user, setUser] = useState<EmbeddedUser | null>(null);
  const [entered, setEntered] = useState(false);
  const [presets, setPresets] = useState<Preset[]>(emptyPresets);
  const [selected, setSelected] = useState(0);
  const [block, setBlock] = useState<BlockKind>('nam');
  const [loading, setLoading] = useState(false);
  const [checking, setChecking] = useState(false);
  const [browsing, setBrowsing] = useState(false);
  const [flashOpen, setFlashOpen] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const popup = useRef<Window | null>(null);
  // Which block the open select popup is for.
  const target = useRef<{ preset: number; kind: BlockKind }>({ preset: 0, kind: 'nam' });

  const updatePreset = (i: number, fn: (p: Preset) => Preset) =>
    setPresets((prev) => prev.map((p, j) => (j === i ? fn(p) : p)));

  // Download and validate a .nam; store the parsed JSON on the block or report why not.
  const checkNam = async (i: number, model: Model) => {
    setChecking(true);
    setError(null);
    try {
      const res = await t3kClient.fetchFile(model.model_url);
      if (!res.ok) throw new Error(`Download failed: ${res.status}`);
      const { namJson, numWeights, architecture } = analyzeNamFile(await res.text());
      if (!isPedalCompatible(numWeights)) {
        throw new Error(`${architecture} model with ${numWeights} weights. The pedal runs A2 nano models only (${PEDAL_WEIGHT_COUNT} weights).`);
      }
      updatePreset(i, (p) => (p.nam?.model.id === model.id ? { ...p, nam: { ...p.nam, namJson } } : p));
    } catch (err) {
      setError(`${model.name}: ${(err as Error).message}`);
    } finally {
      setChecking(false);
    }
  };

  const setModel = (model: Model) => {
    const i = selected;
    if (block === 'nam') {
      updatePreset(i, (p) => (p.nam ? { ...p, nam: { ...p.nam, model, namJson: null } } : p));
      checkNam(i, model);
    } else {
      updatePreset(i, (p) => (p.ir ? { ...p, ir: { ...p.ir, model } } : p));
    }
  };

  // Restore a stored session.
  useEffect(() => {
    if (!t3kClient.isConnected()) return;
    t3kClient.getUser().then(setUser).catch(() => t3kClient.clearTokens());
  }, []);

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

      const { preset: i, kind } = target.current;
      setLoading(true);
      try {
        const [tone, list] = await Promise.all([
          t3kClient.getTone(result.toneId),
          t3kClient.listModels(result.toneId),
        ]);
        const models = list.data.filter(kind === 'nam' ? isNamFile : isIrFile);
        if (models.length === 0) throw new Error(kind === 'nam' ? 'No A2 models in this tone.' : 'No IR files in this tone.');
        const blk = { tone, models, model: models[0] };
        if (kind === 'nam') {
          updatePreset(i, (p) => ({ ...p, nam: { ...blk, namJson: null } }));
          setLoading(false);
          await checkNam(i, models[0]);
        } else {
          updatePreset(i, (p) => ({ ...p, ir: blk }));
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
  const select = (kind: BlockKind) => {
    target.current = { preset: selected, kind };
    open(SELECT_OPTIONS[kind]);
  };
  const remove = (kind: BlockKind) => updatePreset(selected, (p) => ({ ...p, [kind]: null }));

  const preset = presets[selected];
  const detail = preset[block] ?? preset.nam ?? preset.ir;
  const detailKind: BlockKind = detail === preset.nam ? 'nam' : 'ir';

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
            onSelect={(i) => {
              setSelected(i);
              setBlock('nam');
            }}
            onFlash={() => setFlashOpen(true)}
          />
        </main>
      )}

      {flashOpen && <FlashDialog presets={presets} onClose={() => setFlashOpen(false)} />}
    </>
  );
}
