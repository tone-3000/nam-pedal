// Preset chain, after the plugin's gallery: [NAM] - [IR]. Empty blocks are add tiles.
import { ArrowLeftRight, CirclePlus, Trash2 } from 'lucide-react';
import type { Preset, BlockKind, Block } from '../lib/flash';
import { CrossOriginImage } from './CrossOriginImage';

const LABEL: Record<BlockKind, string> = { nam: 'NAM', ir: 'IR' };

interface TileProps {
  kind: BlockKind;
  block: Block | null;
  active: boolean;
  dim: boolean;
  onOpen: () => void;
  onSwap: () => void;
  onRemove: () => void;
}

function Tile({ kind, block, active, dim, onOpen, onSwap, onRemove }: TileProps) {
  if (!block) {
    return (
      <button className={`tile tile-add tile-add-${kind}`} onClick={onSwap} aria-label={`Add ${LABEL[kind]}`}>
        <span className="badge tile-badge">{LABEL[kind]}</span>
        <CirclePlus size={48} strokeWidth={1} />
      </button>
    );
  }
  const image = block.tone.images?.[0];
  return (
    <div
      className={`tile${active ? ' active' : ''}${dim ? ' dim' : ''}`}
      role="button"
      tabIndex={0}
      onClick={onOpen}
      onKeyDown={(e) => e.key === 'Enter' && onOpen()}
    >
      {image ? <CrossOriginImage src={image} alt={block.tone.title} className="tile-image" /> : <div className="tile-image" />}
      <div className="tile-chrome" onClick={(e) => e.stopPropagation()}>
        <span className="badge">{LABEL[kind]}</span>
        <span className="tile-chrome-right">
          <button className="btn-icon plain" onClick={onSwap} aria-label={`Swap ${LABEL[kind]}`}>
            <ArrowLeftRight size={14} />
          </button>
          <button className="btn-icon plain" onClick={onRemove} aria-label={`Remove ${LABEL[kind]}`}>
            <Trash2 size={14} />
          </button>
        </span>
      </div>
    </div>
  );
}

interface Props {
  preset: Preset;
  active: BlockKind;
  checking: boolean;
  onOpen: (kind: BlockKind) => void;
  onSwap: (kind: BlockKind) => void;
  onRemove: (kind: BlockKind) => void;
}

export function Chain({ preset, active, checking, onOpen, onSwap, onRemove }: Props) {
  const tile = (kind: BlockKind) => (
    <Tile
      kind={kind}
      block={preset[kind]}
      active={active === kind && preset[kind] !== null}
      dim={kind === 'nam' && (checking || preset.nam?.namJson == null)}
      onOpen={() => onOpen(kind)}
      onSwap={() => onSwap(kind)}
      onRemove={() => onRemove(kind)}
    />
  );
  return (
    <div className="chain">
      {tile('nam')}
      <span className="chain-link" />
      {tile('ir')}
    </div>
  );
}
