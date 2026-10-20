import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { T3kPlayerProvider } from 'neural-amp-modeler-wasm';
import './index.css';
import App from './App.tsx';

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <T3kPlayerProvider>
      <App />
    </T3kPlayerProvider>
  </StrictMode>,
);
