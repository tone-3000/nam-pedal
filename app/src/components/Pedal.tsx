// Pedal panel: the rotary section of the pedal (band, bezels, LEDs, knob) plus the flash and sync buttons.
//
// The face is drawn flat (black, LED track left bare). The knob and one chrome bezel with its LED off / lit
// are sprites rendered from the enclosure model by hardware/renders/src/app_face.py; face.json carries the
// geometry in face millimetres, which is also the SVG unit here.
import { useEffect, useRef, useState } from 'react';
import { Download, Zap } from 'lucide-react';
import type { Preset } from '../lib/flash';
import { isReady } from '../lib/flash';
import { isWebSerialSupported } from '../lib/serial';
import { Spinner } from './Spinner';
import geom from '../assets/pedal/face.json';
import knobUrl from '../assets/pedal/knob.webp';
import bezelOffUrl from '../assets/pedal/bezel-off.webp';
import bezelOnUrl from '../assets/pedal/bezel-on.webp';

const { band, tiles } = geom;
const [RX, RY] = geom.rotary.center;
// Detents, degrees clockwise from 12 o'clock. Deliberately wider than the drill template
// (±44° / ±15°, see face.json) so the four positions read clearly at panel size.
const ANGLES = [-62, -21, 21, 62];
const LED_R = (band.outer + band.inner) / 2; // LEDs sit on the band's centre line
const SWEEP = 75; // drag limit past the end detents

// Crop of the face shown in the panel (mm, template coordinates: origin top-left of the print, y down).
const VX = 51;
const VY = 47;
const VW = 79.5;
const VH = 60;

const KNOB_HIT_R = 24; // grab disc; the bezels start ~27 mm from the axis
const LED_HIT_R = 5;

const LEDS = ANGLES.map((deg) => {
  const a = (deg * Math.PI) / 180;
  return { center: [RX + LED_R * Math.sin(a), RY - LED_R * Math.cos(a)] as const };
});

const nearest = (deg: number) =>
  ANGLES.reduce((best, a, i) => (Math.abs(a - deg) < Math.abs(ANGLES[best] - deg) ? i : best), 0);

// Half annulus, flat ends on the rotary centre line.
const BAND_PATH = [
  `M ${RX - band.outer} ${RY}`,
  `A ${band.outer} ${band.outer} 0 0 1 ${RX + band.outer} ${RY}`,
  `L ${RX + band.inner} ${RY}`,
  `A ${band.inner} ${band.inner} 0 0 0 ${RX - band.inner} ${RY}`,
  'Z',
].join(' ');

interface FaceProps {
  selected: number;
  onSelect: (i: number) => void;
}

function Face({ selected, onSelect }: FaceProps) {
  const svg = useRef<SVGSVGElement>(null);
  const [drag, setDrag] = useState<number | null>(null);
  const angle = drag ?? ANGLES[selected];

  // Track the pointer on window while dragging so the knob follows even outside the SVG.
  useEffect(() => {
    if (drag === null) return;
    const onMove = (e: PointerEvent) => {
      const p = new DOMPoint(e.clientX, e.clientY).matrixTransform(svg.current!.getScreenCTM()!.inverse());
      const a = Math.max(-SWEEP, Math.min(SWEEP, (Math.atan2(p.x - RX, RY - p.y) * 180) / Math.PI));
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

  const half = tiles.bezel / 2;
  const kh = tiles.knob / 2;

  return (
    <svg ref={svg} className="face" viewBox={`${VX} ${VY} ${VW} ${VH}`} role="group" aria-label="Preset selector">
      <defs>
        <filter id="knob-shadow" x="-20%" y="-20%" width="140%" height="140%">
          <feDropShadow dx="0.3" dy="0.9" stdDeviation="0.7" floodColor="#000" floodOpacity="0.55" />
        </filter>
      </defs>

      <rect x={VX} y={VY} width={VW} height={VH} className="face-ink" />
      <path d={BAND_PATH} className="face-alu" />

      {LEDS.map((led, i) => {
        const [x, y] = led.center;
        const on = i === selected;
        return (
          <g key={i} className="face-hit" role="button" aria-label={`Preset ${i + 1}`} aria-pressed={on} onClick={() => onSelect(i)}>
            <image href={bezelOffUrl} x={x - half} y={y - half} width={tiles.bezel} height={tiles.bezel} />
            <image href={bezelOnUrl} x={x - half} y={y - half} width={tiles.bezel} height={tiles.bezel} className={`face-lit${on ? ' on' : ''}`} />
            <circle cx={x} cy={y} r={LED_HIT_R} fill="transparent" />
          </g>
        );
      })}

      <g filter="url(#knob-shadow)">
        <image
          href={knobUrl}
          x={RX - kh}
          y={RY - kh}
          width={tiles.knob}
          height={tiles.knob}
          className={`face-knob-img${drag !== null ? ' dragging' : ''}`}
          style={{ transform: `rotate(${angle - geom.rotary.modelAngle}deg)`, transformOrigin: `${RX}px ${RY}px` }}
        />
      </g>
      <circle
        cx={RX}
        cy={RY}
        r={KNOB_HIT_R}
        fill="transparent"
        className={`face-knob${drag !== null ? ' dragging' : ''}`}
        role="slider"
        aria-label="Preset knob"
        aria-valuemin={1}
        aria-valuemax={ANGLES.length}
        aria-valuenow={selected + 1}
        onPointerDown={() => setDrag(angle)}
      />
    </svg>
  );
}

interface Props {
  presets: Preset[];
  selected: number;
  syncing: boolean;
  onSelect: (i: number) => void;
  onFlash: () => void;
  onSync: () => void;
}

export function Pedal({ presets, selected, syncing, onSelect, onFlash, onSync }: Props) {
  const count = presets.filter(isReady).length;
  const canSync = isWebSerialSupported();

  return (
    <aside className="card">
      <div className="section-title">
        <span className="label">Pedal</span>
        <span className="label">{count}/{presets.length}</span>
      </div>
      <Face selected={selected} onSelect={onSelect} />
      <button className="btn btn-primary btn-block pedal-flash" disabled={count === 0 || syncing} onClick={onFlash}>
        <Zap size={14} />
        Flash to pedal
      </button>
      <button
        className="btn btn-block pedal-sync"
        disabled={!canSync || syncing}
        onClick={onSync}
        title={canSync ? 'Load the presets currently on the plugged-in pedal' : 'Web Serial is not supported in this browser; use Chrome or Edge.'}
      >
        {syncing ? <Spinner /> : <Download size={14} />}
        {syncing ? 'Reading pedal' : 'Sync from pedal'}
      </button>
    </aside>
  );
}
