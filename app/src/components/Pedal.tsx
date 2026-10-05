// Pedal panel: faceplate with the rotary preset knob and 4 red LEDs, plus the flash button.
import { useEffect, useRef, useState } from 'react';
import { Zap } from 'lucide-react';
import type { Preset } from '../lib/flash';
import { isReady } from '../lib/flash';

// Faceplate geometry in SVG units, after the pedal wireframe: an LED arc band over the knob.
const VIEW_W = 300;
const VIEW_H = 232;
const CX = 150;
const CY = 162;
const BAND_OUTER = 134;
const BAND_INNER = 100;
const LED_R = 117;
const LABEL_R = 146;
const BOSS_R = 40;
const POINTER_W = 34;
const TIP = 95; // pointer reach from center, just short of the band's inner edge
const TAIL = 58; // pointer overhang past the center
const ANGLES = [-70, -24, 24, 70]; // degrees from 12 o'clock
const SWEEP = 85; // drag limit past the end positions

const polar = (r: number, deg: number) => {
  const a = (deg * Math.PI) / 180;
  return { x: CX + r * Math.sin(a), y: CY - r * Math.cos(a) };
};

const nearest = (deg: number) =>
  ANGLES.reduce((best, a, i) => (Math.abs(a - deg) < Math.abs(ANGLES[best] - deg) ? i : best), 0);

// Half annulus, flat ends on the knob's center line.
const BAND_PATH = [
  `M ${CX - BAND_OUTER} ${CY}`,
  `A ${BAND_OUTER} ${BAND_OUTER} 0 0 1 ${CX + BAND_OUTER} ${CY}`,
  `L ${CX + BAND_INNER} ${CY}`,
  `A ${BAND_INNER} ${BAND_INNER} 0 0 0 ${CX - BAND_INNER} ${CY}`,
  'Z',
].join(' ');


interface FaceProps {
  selected: number;
  ready: boolean[];
  onSelect: (i: number) => void;
}

function Face({ selected, ready, onSelect }: FaceProps) {
  const svg = useRef<SVGSVGElement>(null);
  const [drag, setDrag] = useState<number | null>(null);
  const angle = drag ?? ANGLES[selected];

  // Track the pointer on window while dragging so the knob follows even outside the SVG.
  useEffect(() => {
    if (drag === null) return;
    const onMove = (e: PointerEvent) => {
      const r = svg.current!.getBoundingClientRect();
      const s = r.width / VIEW_W;
      const dx = e.clientX - (r.left + CX * s);
      const dy = e.clientY - (r.top + CY * s);
      const a = Math.max(-SWEEP, Math.min(SWEEP, (Math.atan2(dx, -dy) * 180) / Math.PI));
      setDrag(a);
      const i = nearest(a);
      if (i !== selected) onSelect(i);
    };
    const onUp = () => setDrag(null);
    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
    window.addEventListener('pointercancel', onUp);
    return () => {
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
      window.removeEventListener('pointercancel', onUp);
    };
  }, [drag, selected, onSelect]);

  return (
    <svg ref={svg} className="face" viewBox={`0 0 ${VIEW_W} ${VIEW_H}`} role="group" aria-label="Preset selector">
      <rect width={VIEW_W} height={VIEW_H} rx={14} className="face-bg" />
      <image href="/brand/t3k.svg" x={16} y={16} height={12} />
      <path d={BAND_PATH} className="face-band" />
      {ANGLES.map((deg, i) => {
        const led = polar(LED_R, deg);
        const label = polar(LABEL_R, deg);
        const on = i === selected;
        return (
          <g key={i} className="hit" onClick={() => onSelect(i)} role="button" aria-label={`Preset ${i + 1}`}>
            <circle cx={led.x} cy={led.y} r={16} fill="transparent" />
            <circle cx={led.x} cy={led.y} r={7} className={`led${on ? ' on' : ''}`} />
            <text x={label.x} y={label.y + 4} className={`led-label${ready[i] ? ' ready' : ''}`}>
              {i + 1}
            </text>
          </g>
        );
      })}
      <g
        className={`knob${drag !== null ? ' dragging' : ''}`}
        role="slider"
        aria-label="Preset knob"
        aria-valuemin={1}
        aria-valuemax={ANGLES.length}
        aria-valuenow={selected + 1}
        transform={`rotate(${angle} ${CX} ${CY})`}
        onPointerDown={() => setDrag(angle)}
      >
        <circle cx={CX} cy={CY} r={BOSS_R} className="knob-body" />
        <rect x={CX - POINTER_W / 2} y={CY - TIP} width={POINTER_W} height={TIP + TAIL} rx={10} className="knob-body" />
        <rect x={CX - 2.5} y={CY - TIP + 8} width={5} height={30} rx={2.5} className="knob-stripe" />
      </g>
    </svg>
  );
}

interface Props {
  presets: Preset[];
  selected: number;
  onSelect: (i: number) => void;
  onFlash: () => void;
}

export function Pedal({ presets, selected, onSelect, onFlash }: Props) {
  const ready = presets.map(isReady);
  const count = ready.filter(Boolean).length;

  return (
    <aside className="card">
      <div className="section-title">
        <span className="label">Pedal</span>
        <span className="label">{count}/{presets.length}</span>
      </div>
      <Face selected={selected} ready={ready} onSelect={onSelect} />
      <button className="btn btn-primary btn-block pedal-flash" disabled={count === 0} onClick={onFlash}>
        <Zap size={14} />
        Flash to pedal
      </button>
    </aside>
  );
}
