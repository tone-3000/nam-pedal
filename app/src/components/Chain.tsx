// Preset chain, after the plugin's gallery: [NAM] - [NAM | IR]. Empty slots are add tiles.
import { ArrowLeftRight, CirclePlus, Trash2 } from 'lucide-react';
import type { Preset, BlockKind, Block, Slot } from '../lib/flash';
import { slotKind } from '../lib/flash';
import { CrossOriginImage } from './CrossOriginImage';

const LABEL: Record<BlockKind, string> = { nam: 'NAM', nam2: 'NAM', ir: 'IR' };
// Empty-slot labels: slot 2 takes either a second capture or a cab IR.
const SLOT_LABEL: Record<Slot, string> = { 1: 'NAM', 2: 'NAM / IR' };

interface TileProps {
  slot: Slot;
  kind: BlockKind | null;
  block: Block | null;
  active: boolean;
  dim: boolean;
  onOpen: () => void;
  onSwap: () => void;
  onRemove: () => void;
}

function Tile({ slot, kind, block, active, dim, onOpen, onSwap, onRemove }: TileProps) {
  if (!block || !kind) {
    return (
      <button className={`tile tile-add tile-add-${slot}`} onClick={onSwap} aria-label={`Add ${SLOT_LABEL[slot]}`}>
        <span className="badge tile-badge">{SLOT_LABEL[slot]}</span>
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
  onSwap: (slot: Slot) => void;
  onRemove: (kind: BlockKind) => void;
}

export function Chain({ preset, active, checking, onOpen, onSwap, onRemove }: Props) {
  const tile = (slot: Slot) => {
    const kind = slotKind(preset, slot);
    return (
      <Tile
        slot={slot}
        kind={kind}
        block={kind && preset[kind]}
        active={kind !== null && active === kind}
        dim={kind !== null && kind !== 'ir' && (checking || preset[kind]?.namJson == null)}
        onOpen={() => kind && onOpen(kind)}
        onSwap={() => onSwap(slot)}
        onRemove={() => kind && onRemove(kind)}
      />
    );
  };
  return (
    <div className="chain">
      {tile(1)}
      <span className="chain-link" />
      {tile(2)}
    </div>
  );
}
