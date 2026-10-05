// TONE3000 OAuth (PKCE, select-tone popup flow) and authenticated API client.

import { T3K_API } from './config';
import type { EmbeddedUser, Tone, Model, PaginatedResponse } from './types';

export interface T3KTokens {
  access_token: string;
  refresh_token: string;
  expires_at: number; // Unix ms
}

export type OAuthCallbackResult =
  | { ok: true; tokens: T3KTokens; toneId?: string; canceled?: boolean }
  | { ok: false; error: string };

export interface OAuthPopupOptions {
  prompt?: 'select_tone'; // omit for plain login
  gears?: string; // underscore-separated, e.g. 'amp_pedal'
  format?: 'nam' | 'ir';
  architecture?: number;
  menubar?: boolean;
  preview?: boolean; // audition players in the select flow
}

// PKCE

const base64url = (bytes: ArrayBuffer | Uint8Array) =>
  btoa(String.fromCharCode(...new Uint8Array(bytes)))
    .replace(/\+/g, '-')
    .replace(/\//g, '_')
    .replace(/=/g, '');

const randomBase64url = (bytes: number) => base64url(crypto.getRandomValues(new Uint8Array(bytes)));

const sha256Base64url = async (input: string) =>
  base64url(await crypto.subtle.digest('SHA-256', new TextEncoder().encode(input)));

async function tokenRequest(body: Record<string, string>): Promise<T3KTokens> {
  const res = await fetch(`${T3K_API}/api/v1/oauth/token`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams(body),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { error?: string }).error ?? 'token_request_failed');
  }
  const data = await res.json();
  return {
    access_token: data.access_token,
    refresh_token: data.refresh_token,
    expires_at: Date.now() + data.expires_in * 1000,
  };
}

// OAuth popup

/** Open TONE3000 login (or the tone browser) in a popup. The result arrives via postMessage or BroadcastChannel. */
export async function startOAuthPopup(
  publishableKey: string,
  redirectUri: string,
  options: OAuthPopupOptions = {},
): Promise<Window | null> {
  // The popup copies sessionStorage on open; clear the flag here so only the popup keeps it.
  sessionStorage.setItem('t3k_popup_mode', '1');

  const codeVerifier = randomBase64url(32);
  const state = randomBase64url(16);
  sessionStorage.setItem('t3k_code_verifier', codeVerifier);
  sessionStorage.setItem('t3k_state', state);

  const url = new URL(`${T3K_API}/api/v1/oauth/authorize`);
  const params: Record<string, string> = {
    client_id: publishableKey,
    redirect_uri: redirectUri,
    response_type: 'code',
    code_challenge: await sha256Base64url(codeVerifier),
    code_challenge_method: 'S256',
    state,
  };
  if (options.prompt) params.prompt = options.prompt;
  if (options.gears) params.gears = options.gears;
  if (options.format) params.format = options.format;
  if (options.architecture) params.architecture = String(options.architecture);
  if (options.menubar) params.menubar = 'true';
  if (options.preview) params.preview = 'true';
  for (const [k, v] of Object.entries(params)) url.searchParams.set(k, v);

  const width = 480;
  const height = 700;
  const left = Math.round(window.screenX + (window.outerWidth - width) / 2);
  const top = Math.round(window.screenY + (window.outerHeight - height) / 2);
  const popup = window.open(
    url.toString(),
    't3k_select',
    `width=${width},height=${height},left=${left},top=${top},toolbar=no,menubar=no,location=no,status=no,resizable=yes,scrollbars=yes`,
  );
  sessionStorage.removeItem('t3k_popup_mode');
  return popup;
}

/** Verify state and exchange the code relayed from the popup. Returns null for unrelated events. */
export async function handleOAuthCallbackFromPopup(
  publishableKey: string,
  redirectUri: string,
  event: MessageEvent,
): Promise<OAuthCallbackResult | null> {
  if (event.data?.type !== 't3k_oauth_callback') return null;
  const { code, state, error, tone_id: toneId, canceled } = event.data;

  const storedState = sessionStorage.getItem('t3k_state');
  const codeVerifier = sessionStorage.getItem('t3k_code_verifier');
  sessionStorage.removeItem('t3k_state');
  sessionStorage.removeItem('t3k_code_verifier');

  if (state !== storedState) return { ok: false, error: 'state_mismatch' };
  if (canceled && !code) return { ok: false, error: 'canceled' };
  if (error) return { ok: false, error };
  if (!code || !codeVerifier) return { ok: false, error: 'missing_code' };

  try {
    const tokens = await tokenRequest({
      grant_type: 'authorization_code',
      code,
      code_verifier: codeVerifier,
      redirect_uri: redirectUri,
      client_id: publishableKey,
    });
    return { ok: true, tokens, toneId, ...(canceled ? { canceled: true } : {}) };
  } catch (err) {
    return { ok: false, error: (err as Error).message };
  }
}

// API client

const STORAGE_KEY = 't3k_tokens';

/** Bearer-authenticated fetch with token refresh. Tokens persist in localStorage. */
export class T3KClient {
  private refreshing: Promise<T3KTokens> | null = null;

  constructor(private readonly publishableKey: string) {}

  setTokens(tokens: T3KTokens): void {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(tokens));
  }

  getTokens(): T3KTokens | null {
    const raw = localStorage.getItem(STORAGE_KEY);
    return raw ? (JSON.parse(raw) as T3KTokens) : null;
  }

  clearTokens(): void {
    localStorage.removeItem(STORAGE_KEY);
  }

  isConnected(): boolean {
    return this.getTokens() !== null;
  }

  private async accessToken(): Promise<string> {
    const tokens = this.getTokens();
    if (!tokens) throw new Error('Not authenticated');
    if (Date.now() < tokens.expires_at - 60_000) return tokens.access_token;

    this.refreshing ??= tokenRequest({
      grant_type: 'refresh_token',
      refresh_token: tokens.refresh_token,
      client_id: this.publishableKey,
    })
      .then((t) => {
        this.setTokens(t);
        return t;
      })
      .catch((err) => {
        this.clearTokens();
        throw err;
      })
      .finally(() => {
        this.refreshing = null;
      });
    return (await this.refreshing).access_token;
  }

  async fetch(path: string, init?: RequestInit): Promise<Response> {
    const send = async () =>
      globalThis.fetch(`${T3K_API}${path}`, {
        ...init,
        headers: { ...init?.headers, Authorization: `Bearer ${await this.accessToken()}` },
      });
    const res = await send();
    if (res.status !== 401) return res;
    // Force a refresh and retry once.
    const stored = this.getTokens();
    if (!stored) return res;
    this.setTokens({ ...stored, expires_at: 0 });
    return send();
  }

  /** model_url is absolute; the API host is stripped so fetch() can add auth. */
  fetchFile(modelUrl: string): Promise<Response> {
    return this.fetch(modelUrl.replace(/^https?:\/\/[^/]+/, ''));
  }

  async getUser(): Promise<EmbeddedUser> {
    const res = await this.fetch('/api/v1/user');
    if (!res.ok) throw new Error(`getUser failed: ${res.status}`);
    return res.json();
  }

  async getTone(id: number | string): Promise<Tone> {
    const res = await this.fetch(`/api/v1/tones/${id}`);
    if (!res.ok) throw new Error(`getTone failed: ${res.status}`);
    return res.json();
  }

  // The pedal runs A2 only. IR files have no architecture and pass through the filter.
  async listModels(toneId: number | string): Promise<PaginatedResponse<Model>> {
    const qs = new URLSearchParams({ tone_id: String(toneId), architecture: '2' });
    const res = await this.fetch(`/api/v1/models?${qs}`);
    if (!res.ok) throw new Error(`listModels failed: ${res.status}`);
    return res.json();
  }
}
