"""Timestamped contact sheet from real 10 Hz camera frames, no generated poses."""
import argparse
from pathlib import Path
import cv2
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('run', type=Path)
p.add_argument('--start', type=float, default=10)
p.add_argument('--step', type=float, default=3)
a = p.parse_args()
canvas = np.full((4 * 257, 3 * 400, 3), 245, np.uint8)
for n in range(12):
    t = a.start + n * a.step
    path = a.run / f'frame_{round(t * 10) + 1:05d}.jpg'
    im = cv2.imread(str(path))
    if im is None:
        raise RuntimeError('Missing real frame: ' + str(path))
    tile = cv2.resize(im, (400, 225), interpolation=cv2.INTER_AREA)
    x, y = (n % 3) * 400, (n // 3) * 257
    canvas[y:y + 225, x:x + 400] = tile
    cv2.putText(canvas, f'Camera time: {t:.1f} s', (x + 10, y + 247), cv2.FONT_HERSHEY_SIMPLEX, .58, (40, 40, 40), 1, cv2.LINE_AA)
cv2.imwrite(str(a.run / 'contact_sheet.jpg'), canvas)
