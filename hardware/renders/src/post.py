"""Crop to alpha bbox + margin on (20,20,20), resize to 1200 wide, JPEG q88. Card 1280x720, hero 1204x1024."""
import sys, os
from PIL import Image

BG = (20, 20, 20)
src_dir, dst_dir = sys.argv[1], sys.argv[2]
os.makedirs(dst_dir, exist_ok=True)

def flat(im, margin=60):
    a = im.split()[3]
    box = a.getbbox()
    im = im.crop((max(0, box[0] - margin), max(0, box[1] - margin), min(im.width, box[2] + margin), min(im.height, box[3] + margin)))
    bg = Image.new("RGB", im.size, BG); bg.paste(im, mask=im.split()[3])
    return bg

def fit(im, w, h, pad=1.08):
    """place the subject centered on a w x h canvas, subject scaled to fit with padding"""
    a = im.split()[3]; box = a.getbbox()
    sub = im.crop(box)
    s = min(w / (sub.width * pad), h / (sub.height * pad))
    sub = sub.resize((int(sub.width * s), int(sub.height * s)), Image.LANCZOS)
    canvas = Image.new("RGB", (w, h), BG)
    canvas.paste(sub, ((w - sub.width) // 2, (h - sub.height) // 2), mask=sub.split()[3])
    return canvas

for f in sorted(os.listdir(src_dir)):
    if not f.endswith(".png"):
        continue
    name = f[:-4]
    im = Image.open(os.path.join(src_dir, f)).convert("RGBA")
    if name == "hero":
        fit(im, 1204, 1024).save(os.path.join(dst_dir, "hero.jpg"), quality=90)
        fit(im, 1280, 720).save(os.path.join(dst_dir, "card.jpg"), quality=90)
    out = flat(im)
    if out.width > 1200:
        out = out.resize((1200, round(out.height * 1200 / out.width)), Image.LANCZOS)
    out.save(os.path.join(dst_dir, name + ".jpg"), quality=88)
    print(name, out.size)
