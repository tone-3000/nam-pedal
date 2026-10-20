# TONE3000 DIY Pedal Loader

Companion web app for the TONE3000 × Daisy DIY Pedal (T3K Pedal). It loads NAM
captures and cab IRs from [TONE3000](https://www.tone3000.com) onto the pedal's
four presets over USB.

## Flow

1. **Log in.** TONE3000 OAuth (PKCE) in a popup. The session is stored, so returning
   users go straight to the presets.
2. **Pick a preset.** Drag the on-screen rotary knob (or click an LED) to choose
   preset 1 to 4.
3. **Build the chain.** Each preset is one or two blocks: an A2-Lite NAM capture on
   block 1, and on block 2 either a second A2-Lite capture (e.g. a drive pedal into
   an amp) or a cab IR. Clicking an empty block opens the TONE3000 select flow:
   block 1 is filtered to A2 NAM tones, block 2 to NAM and IR tones of every gear
   except spaces, and the tone's files decide which it becomes. NAM models are
   validated against the pedal's fixed A2-Lite architecture (1871 weights). Click a filled
   block to step through the tone's models and preview them in the browser
   (`neural-amp-modeler-wasm`), swap the tone, or remove it.
4. **Flash.** The app converts each `.nam` to `.namb`, packs a 4-entry bank (format
   v2) in rotary order (empty presets are zero-size entries), and writes it to QSPI
   at `0x90600000` over WebUSB (ST DfuSe). The pedal loads whichever preset the
   rotary points at and lights the LED above it.

Preset state is kept in `localStorage`, so coming back in the same browser picks up
where you left off (the `.nam` files are re-downloaded and re-validated on return).
**Sync from pedal** rebuilds the presets from whatever is on a plugged-in pedal: the
bank stores each block's TONE3000 model id, and the app asks the firmware for the
bank directory over Web Serial (`'B'` command), then resolves each id back to its
tone (Get Model → Get Tone → List Models). Blocks that can't be linked (banks packed
from local files without ids, or tones no longer accessible) are left empty, with a
note in the banner. Older firmware without the `'B'` command times out with a message
to update.

## Requirements

- Chrome or Edge (WebUSB and Web Serial).
- Pedal running the T3K Pedal firmware with the Daisy bootloader
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

- `src/App.tsx`: auth, preset chains, tone loading and model validation, sync and restore.
- `src/components/`: `Splash`, `Header`, `Chain` (NAM / NAM-or-IR tiles), `BlockDetail` (tone
  details + model selector), `Pedal` (faceplate, knob, LEDs, flash and sync buttons), `FlashDialog`.
- `src/assets/pedal/`: knob and bezel/LED sprites rendered from the enclosure model plus
  `face.json` (face geometry in mm). Regenerate with
  `blender -b -P hardware/renders/src/app_face.py` after changing the model.
- `src/tone3000-client.ts`: PKCE select flow and authenticated API client.
- `src/lib/namb.ts`: port of nam-binary-loader `nam2namb`.
- `src/lib/bank.ts`: port of `pack_models.py` (bank layout, IR pipeline).
- `src/lib/webdfu.ts`: WebUSB port of `pydfu.py`.
- `src/lib/serial.ts`: Web Serial DFU trigger and bank directory reader.
- `src/lib/sync.ts`: pedal bank directory → presets (TONE3000 id resolution).
- `src/lib/store.ts`: `localStorage` persistence of the presets.

## Design

Black and greys with white pill buttons, as in the plugin. Pure red for the LEDs,
pure yellow only for small accents. Rounded cards and tiles, no hover effects.
Arial body, Roboto Mono labels. The faceplate is drawn flat from the drill template
geometry (black print, bare-aluminium LED track) with the knob and chrome bezels rendered
from the 3D model. Logos are
from the [TONE3000 API design requirements](https://www.tone3000.com/api#design-requirements).
Icons are [Lucide](https://lucide.dev). COOP/COEP headers in `vite.config.ts` are
required by the WASM preview player (SharedArrayBuffer).
