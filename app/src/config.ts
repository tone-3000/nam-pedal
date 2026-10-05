// Trailing slashes stripped: a double slash 308-redirects and loses CORS headers.
export const T3K_API = (
  (import.meta.env.VITE_T3K_API_DOMAIN as string | undefined) ?? 'https://www.tone3000.com'
).replace(/\/+$/, '');

export const PUBLISHABLE_KEY = import.meta.env.VITE_PUBLISHABLE_KEY as string;

export const REDIRECT_URI =
  (import.meta.env.VITE_REDIRECT_URI as string | undefined) ?? 'http://localhost:3001';
