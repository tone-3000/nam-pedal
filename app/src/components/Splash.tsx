import { Tone3000Logo } from './Brand';

interface Props {
  signedIn: boolean;
  busy: boolean;
  onContinue: () => void;
}

export function Splash({ signedIn, busy, onContinue }: Props) {
  const cta = busy ? 'Waiting for TONE3000' : signedIn ? 'Select Presets' : 'Log in or Create Account';
  return (
    <section className="splash">
      <div className="splash-brand">
        <Tone3000Logo className="splash-logo" />
        <span className="splash-pedal">DIY PEDAL</span>
      </div>
      <p className="splash-copy">
        Load NAM captures and cab IRs from TONE3000 onto the four presets of your DIY pedal over USB.
        Open source hardware and firmware for makers.
      </p>
      <button className="btn btn-primary" disabled={busy} onClick={onContinue}>
        {cta}
      </button>
    </section>
  );
}
