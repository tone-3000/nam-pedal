"""Face / side print textures (base colour + metal-rough) from the enclosure artwork. Run from this directory."""
import os; os.chdir(os.path.dirname(os.path.abspath(__file__)))
"""Build the face textures from the designer's print layer (layer-MC0.png, portrait 20 px/mm)."""
from PIL import Image, ImageFilter
import numpy as np

src = Image.open("../../enclosure/face-artwork.png").convert("L")
land = src.rotate(90, expand=True)            # 2820 x 2341 = 141 x 117 mm, X right, Y down
land = land.resize((2048, int(2048 * 2341 / 2820)), Image.LANCZOS)
ink = 1.0 - np.asarray(land, dtype=np.float32) / 255.0   # 1 = black print
ink = np.clip(ink, 0, 1)[..., None]

alu = np.array([0.78, 0.79, 0.81], dtype=np.float32)
paint = np.array([0.012, 0.012, 0.013], dtype=np.float32)
base = (1 - ink) * alu + ink * paint
Image.fromarray((base * 255).astype(np.uint8), "RGB").save("textures/face_base.png")

# glTF metallicRoughness: G = roughness, B = metallic
rough = (1 - ink) * 0.38 + ink * 0.68
metal = (1 - ink) * 1.0 + ink * 0.0
mr = np.concatenate([np.zeros_like(rough), rough, metal], axis=2)
Image.fromarray((mr * 255).astype(np.uint8), "RGB").save("textures/face_mr.png")
print("ok", base.shape)

# ---- side wordmark (front wall): side1.png is 32 x 135 mm portrait at 20 px/mm; rotate CCW so it reads left-to-right
side = Image.open("../../enclosure/side-wordmark.png").convert("L").rotate(90, expand=True)   # 2700 x 640 = 135 x 32 mm
ink = 1.0 - np.asarray(side, dtype=np.float32) / 255.0
ink = np.clip(ink, 0, 1)[..., None]
base = (1 - ink) * alu + ink * paint
Image.fromarray((base * 255).astype(np.uint8), "RGB").save("textures/side_base.png")
rough = (1 - ink) * 0.38 + ink * 0.68
metal = (1 - ink) * 1.0 + ink * 0.0
mr = np.concatenate([np.zeros_like(rough), rough, metal], axis=2)
Image.fromarray((mr * 255).astype(np.uint8), "RGB").save("textures/side_mr.png")
print("side ok", base.shape)
