import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// COOP/COEP headers are required for SharedArrayBuffer, which the
// neural-amp-modeler-wasm audio preview player depends on.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 3001,
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'credentialless',
    },
  },
  preview: {
    port: 3001,
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'credentialless',
    },
  },
})
