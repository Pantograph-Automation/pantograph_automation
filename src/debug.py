#!/usr/bin/env python3
"""
Simple capture: grab RGB still from Picamera2, keep only bright red spots, save image.
"""
import sys
from picamera2 import Picamera2
import numpy as np
import cv2

OUT = sys.argv[1] if len(sys.argv) > 1 else "red_spots.png"
MIN_RED = 130      # minimum red intensity (0-255)
R_G_RATIO = 1.3     # R must be > R_G_RATIO * G
R_B_RATIO = 1.3    # R must be > R_B_RATIO * B

pc = Picamera2()
pc.configure(pc.create_still_configuration())   # default processed RGB "main"
pc.start()
img = pc.capture_array("main")  # HxWx3 RGB uint8
pc.stop()

r = img[..., 0].astype(np.int16)
g = img[..., 1].astype(np.int16)
b = img[..., 2].astype(np.int16)

mask = (r >= MIN_RED) & (r >= (R_G_RATIO * g)) & (r >= (R_B_RATIO * b))
out = np.zeros_like(img)
out[..., 0] = (img[..., 0] * mask).astype(np.uint8)  # keep red channel where mask true

# save (OpenCV expects BGR)
cv2.imwrite(OUT, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
print(f"Saved {OUT}")
