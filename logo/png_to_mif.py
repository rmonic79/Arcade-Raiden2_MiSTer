#!/usr/bin/env python3
# Converte LogoJoystickCentraleNoAlpha.png 48x48 in .mif 2bpp (4 colori).
# Output: logo.mif (2304 word x 2 bit) + palette stampata.

from PIL import Image
import sys
from collections import Counter

src = "LogoJoystickCentraleNoAlpha.png"
img = Image.open(src).convert("RGB")
print(f"size: {img.size}, mode: {img.mode}")

# Conta colori unici
pixels = list(img.getdata())
counter = Counter(pixels)
print(f"unique colors: {len(counter)}")
for c, n in counter.most_common():
    print(f"  {c}: {n} px")

# Mappa i 4 colori più frequenti -> idx 0..3
top4 = [c for c, _ in counter.most_common(4)]
while len(top4) < 4:
    top4.append((0,0,0))
pal = {c: i for i, c in enumerate(top4)}

W, H = img.size

with open("logo.mem", "w") as f:
    for y in range(H):
        for x in range(W):
            c = pixels[y*W + x]
            if c in pal:
                idx = pal[c]
            else:
                idx = min(pal.items(), key=lambda kv: sum((a-b)**2 for a,b in zip(kv[0],c)))[1]
            f.write(f"{idx:02b}\n")

# Stampa palette in formato Verilog
print("\n// Palette (RGB888):")
for i, c in enumerate(top4):
    r, g, b = c
    print(f"// pal[{i}] = 24'h{r:02X}{g:02X}{b:02X}  // RGB({r},{g},{b})")
print(f"\nGenerated logo.mif: {W*H} entries x 2 bit = {W*H*2} bits ({(W*H*2)//1024}.{((W*H*2)%1024)//100} kbit)")
