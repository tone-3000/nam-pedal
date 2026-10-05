# TONE3000 DIY Pedal Loader

Web app that loads NAM models from [TONE3000](https://www.tone3000.com) onto the
four presets of the DIY pedal (Daisy Seed) over USB.

## Flow

1. **Log in.** TONE3000 OAuth (PKCE) in a popup. The session is stored, so returning
   users go straight to the presets.
2. **Pick a preset.** Drag the on-screen rotary knob (or click an LED) to choose
   preset 1 to 4.
3. **Build the chain.** Each preset is a two-block chain like the plugin: NAM on
   block 1, optional cab IR on block 2. Clicking an empty block opens the TONE3000
   select flow (filtered to A2 NAM tones, or cab IRs); the first model is loaded and
   validated against the pedal's fixed A2 nano architecture (1871 weights). Click a
   filled block to step through the tone's models and preview them in the browser
   (`neural-amp-modeler-wasm`), swap the tone, or remove it.
4. **Flash.** The app converts each `.nam` to `.namb`, packs a 4-entry bank in rotary
   order (empty presets are zero-size entries), and writes it to QSPI at `0x90600000`
   over WebUSB (ST DfuSe). The pedal loads whichever preset the rotary points at and
   lights the LED above it.

## Requirements

- Chrome or Edge (WebUSB and Web Serial).
- Pedal running the NAMPedal firmware with the Daisy bootloader
  (`make program-boot`, `make program-dfu`).
- A TONE3000 publishable key (`t3k_pub_...`) from Settings > API Keys with
  `http://localhost:3001` as a registered redirect URI.

## Setup

```bash
cd app
cp env.example .env   # fill in VITE_PUBLISHABLE_KEY
npm install
npm run dev           # http://localhost:3001
```

## Flashing

1. **Flash to pedal** builds the bank image.
2. **DFU mode:** with the pedal plugged in and running, **Send DFU trigger** and pick
   its serial port. The firmware gets a `'D'` byte, blinks the LEDs twice, and resets
   into the bootloader. Or hold BOOT and tap RESET on the Seed.
3. **Connect and flash:** pick the `DFU in FS Mode` device. The app erases and writes
   the bank region, then reboots the pedal.

## Code

- `src/App.tsx`: auth, preset chains, tone loading and model validation.
- `src/components/`: `Splash`, `Header`, `Chain` (NAM/IR tiles), `BlockDetail` (tone
  details + model selector), `Pedal` (faceplate, knob, LEDs, flash), `FlashDialog`.
- `src/tone3000-client.ts`: PKCE select flow and authenticated API client.
- `src/lib/namb.ts`: port of nam-binary-loader `nam2namb`.
- `src/lib/bank.ts`: port of `pack_models.py` (bank layout, IR pipeline).
- `src/lib/webdfu.ts`: WebUSB port of `pydfu.py`.
- `src/lib/serial.ts`: Web Serial DFU trigger.

## Design

Black and greys with white pill buttons, as in the plugin. Pure red for the LEDs,
pure yellow only for small accents. Rounded cards and tiles, no hover effects.
Arial body, Roboto Mono labels. The faceplate follows the pedal wireframe. Logos are
from the [TONE3000 API design requirements](https://www.tone3000.com/api#design-requirements).
Icons are [Lucide](https://lucide.dev). COOP/COEP headers in `vite.config.ts` are
required by the WASM preview player (SharedArrayBuffer).
