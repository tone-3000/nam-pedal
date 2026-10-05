// TONE3000 API types used by the app.

export type Gear = 'amp' | 'full-rig' | 'pedal' | 'outboard' | 'ir';
export type Platform = 'nam' | 'ir' | 'aida-x' | 'aa-snapshot' | 'proteus';
export type Size = 'standard' | 'lite' | 'feather' | 'nano' | 'custom';

export interface EmbeddedUser {
  id: string;
  username: string;
  avatar_url: string | null;
  url: string;
}

export interface Tone {
  id: number;
  user: EmbeddedUser;
  title: string;
  description: string | null;
  gear: Gear;
  images: string[] | null;
  platform: Platform;
  sizes: Size[];
  models_count: number;
  downloads_count: number;
  favorites_count: number;
  url: string;
}

export interface Model {
  id: number;
  model_url: string;
  name: string;
  size: Size;
  tone_id: number;
}

export interface PaginatedResponse<T> {
  data: T[];
  page: number;
  page_size: number;
  total: number;
  total_pages: number;
}
