import type { EmbeddedUser } from '../types';
import { Tone3000Logo } from './Brand';
import { CrossOriginImage } from './CrossOriginImage';

interface Props {
  user: EmbeddedUser | null;
}

export function Header({ user }: Props) {
  return (
    <header className="header">
      <div className="header-brand">
        <Tone3000Logo />
        <span className="header-sub">DIY PEDAL</span>
      </div>
      {user && (
        <div className="header-right">
          {user.avatar_url && <CrossOriginImage src={user.avatar_url} alt="" className="avatar" />}
          <span className="muted">@{user.username}</span>
        </div>
      )}
    </header>
  );
}
