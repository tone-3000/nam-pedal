// COEP (needed for the WASM player's SharedArrayBuffer) blocks no-CORS images.
import type { ImgHTMLAttributes } from 'react';

export function CrossOriginImage(props: ImgHTMLAttributes<HTMLImageElement>) {
  return <img {...props} crossOrigin="anonymous" />;
}
