// Tone details and model selector for the active chain block.
import { useCallback, useEffect, useRef } from 'react';
import { ChevronLeft, ChevronRight, Download, Bookmark, Folder } from 'lucide-react';
import { T3kSlimPlayer, DEFAULT_IRS, DEFAULT_INPUTS } from 'neural-amp-modeler-wasm';
import 'neural-amp-modeler-wasm/dist/styles.css';
import type { Model } from '../types';
import type { Block, BlockKind } from '../lib/flash';
import { t3kClient } from '../client';
import { CrossOriginImage } from './CrossOriginImage';

const GEAR: Record<string, string> = {
  amp: 'Amp', 'full-rig': 'Amp + Cab', 'amp-cab': 'Amp + Cab', pedal: 'Pedal', outboard: 'Outboard', cab: 'Cab', ir: 'Cab',
};

const compact = (n: number) => Intl.NumberFormat('en', { notation: 'compact' }).format(n);

interface Props {
  kind: BlockKind;
  block: Block;
  checking: boolean;
  onModel: (model: Model) => void;
}

export function BlockDetail({ kind, block, checking, onModel }: Props) {
  const { tone, models, model } = block;
  const index = models.indexOf(model);
  const blobUrl = useRef<string | null>(null);

  useEffect(() => () => {
    if (blobUrl.current) URL.revokeObjectURL(blobUrl.current);
  }, []);

  // The preview player needs a URL it can fetch without auth, so hand it a blob.
  const getData = useCallback(async () => {
    if (blobUrl.current) URL.revokeObjectURL(blobUrl.current);
    const res = await t3kClient.fetchFile(model.model_url);
    if (!res.ok) throw new Error(`Download failed: ${res.status}`);
    blobUrl.current = URL.createObjectURL(await res.blob());
    return {
      model: { name: model.name, url: blobUrl.current, default: true },
      ir: DEFAULT_IRS[0],
      input: DEFAULT_INPUTS[0],
    };
  }, [model]);

  return (
    <div className="detail">
      <div className="tone">
        {tone.images?.[0] ? (
          <CrossOriginImage src={tone.images[0]} alt="" className="tone-image" />
        ) : (
          <div className="tone-image" />
        )}
        <div className="tone-info">
          <h2 className="tone-title">{tone.title}</h2>
          <div className="tone-row">
            <span>{GEAR[tone.gear] ?? tone.gear}</span>
            <span className="badge">{kind === 'nam' ? 'NAM A2' : 'IR'}</span>
          </div>
          <div className="tone-row">
            <span className="tone-stat"><Download /> {compact(tone.downloads_count)}</span>
            <span className="tone-stat"><Bookmark /> {compact(tone.favorites_count)}</span>
            <span className="tone-stat"><Folder /> {tone.models_count}</span>
          </div>
          <div className="tone-row">
            <span className="tone-creator">
              {tone.user.avatar_url && <CrossOriginImage src={tone.user.avatar_url} alt="" className="avatar" />}
              {tone.user.username}
            </span>
          </div>
          {tone.description && <p className="tone-desc">{tone.description}</p>}
        </div>
      </div>

      <div className="model-select">
        <div className={`model-select-bar${checking ? ' off' : ''}`}>
          <button className="btn-icon plain" disabled={checking || index <= 0} onClick={() => onModel(models[index - 1])} aria-label="Previous model">
            <ChevronLeft size={16} />
          </button>
          <span className="model-select-name">{model.name}</span>
          <button className="btn-icon plain" disabled={checking || index >= models.length - 1} onClick={() => onModel(models[index + 1])} aria-label="Next model">
            <ChevronRight size={16} />
          </button>
          <span className="model-select-count">
            <Folder /> {index + 1}/{models.length}
          </span>
        </div>
        {kind === 'nam' && (
          <div className="neural-amp-modeler">
            <T3kSlimPlayer id={`model-${model.id}`} getData={getData} />
          </div>
        )}
      </div>
    </div>
  );
}
