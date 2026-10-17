"""Compose the branded hero/card: pedal render + assembled PCB render on the TONE3000 stripe background.

usage: python3 hero_compose.py PEDAL.png PCB.png OUTDIR
"""
import sys
from PIL import Image, ImageFilter

pedal_src, pcb_src, outdir = sys.argv[1], sys.argv[2], sys.argv[3]

BLACK, RED, YELLOW, BLUE = (0, 0, 0), (254, 0, 0), (254, 254, 0), (0, 0, 208)


def background(w, h):
    bg = Image.new("RGB", (w, h), BLACK)
    c = w // 2
    for x0, x1, col in ((c - 64, c, RED), (c, c + 64, YELLOW), (c + 64, w, BLUE)):
        bg.paste(col, (x0, 0, x1, h))
    return bg.convert("RGBA")


def cutout(path):
    im = Image.open(path).convert("RGBA")
    bbox = im.getchannel("A").point(lambda v: 255 if v > 140 else 0).getbbox()   # ignore the soft floor shadow
    return im.crop(bbox)


def fit_w(im, w):
    return im.resize((w, round(im.height * w / im.width)), Image.LANCZOS)


def place(canvas, im, x, y, shadow=True):
    if shadow:
        a = im.getchannel("A").point(lambda v: int(v * 0.75))
        sh = Image.new("RGBA", im.size, (0, 0, 0, 0))
        sh.putalpha(a)
        pad = 60
        big = Image.new("RGBA", (im.width + 2 * pad, im.height + 2 * pad), (0, 0, 0, 0))
        big.paste(sh, (pad, pad))
        big = big.filter(ImageFilter.GaussianBlur(22))
        canvas.alpha_composite(big, (x - pad + 6, y - pad + 14))
    canvas.alpha_composite(im, (x, y))


pedal = cutout(pedal_src)
pcb = cutout(pcb_src)

# hero 1204x1024: PCB upper-left, pedal lower-right, overlapping a little
W, H = 1204, 1024
cv = background(W, H)
p = fit_w(pedal, 960)
b = fit_w(pcb, 640)
place(cv, b, 20, 40)
place(cv, p, W - p.width - 10, H - p.height - 40)
cv.convert("RGB").save(f"{outdir}/hero-tall.jpg", quality=90)

# card 1280x720: PCB left, pedal right
W, H = 1280, 720
cv = background(W, H)
p = fit_w(pedal, 780)
b = fit_w(pcb, 600)
place(cv, b, 20, (H - b.height) // 2 - 40)
place(cv, p, W - p.width - 20, H - p.height - 30)
cv.convert("RGB").save(f"{outdir}/card.jpg", quality=90)
print("wrote hero-tall.jpg, card.jpg")
