import atexit
import math
import os
import threading

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:  # pragma: no cover - hardware dependency
    Picamera2 = None

_camera_lock = threading.Lock()
_camera = None

# --- TUNABLE PARAMETERS ---
MASK_RADIUS_RATIO = 0.474
MASK_Y_OFFSET = -72
MASK_X_OFFSET = -65
MIN_CONTOUR_CIRCULARITY = 0.75

def _apply_circular_mask(image, x_offset=0, y_offset=0, radius_ratio=0.45):
    h, w = image.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    center = (w // 2 + x_offset, h // 2 + y_offset)
    radius = int(min(h, w) * radius_ratio)
    cv2.circle(mask, center, radius, 255, -1)
    
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    return masked_image

def _isolate_spots(masked_image):
    """
    Isolate dark circular spots.
    Only the region inside the circular mask is inverted.
    Returns white spots on black background.
    """

    h, w = masked_image.shape[:2]

    # Grayscale + contrast
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)

    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold dark spots normally: dark -> white
    _, spot_mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

    # Create circular ROI mask
    circle_mask = np.zeros((h, w), dtype=np.uint8)

    cx = w // 2 + MASK_X_OFFSET
    cy = h // 2 + MASK_Y_OFFSET
    radius = int(0.99 * MASK_RADIUS_RATIO * min(h, w))

    cv2.circle(circle_mask, (cx, cy), radius, 255, -1)

    # Keep only inverted/white spots inside the circle
    result = cv2.bitwise_and(spot_mask, circle_mask)

    return result

def _clean_noise_morphology(color_mask):
    # Use a slightly larger kernel to bridge gaps in foam texture
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
    
    # Remove noise, then close holes
    opened = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel, iterations=2)
    cleaned_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=4)
    
    return cleaned_mask

def _contour_circularity(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 0.0
    return 4 * math.pi * area / (perimeter * perimeter)

def _extract_centroids(original_image, cleaned_mask, min_area=3000, max_area=1000000):
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    centroids = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (min_area < area < max_area):
            continue

        if _contour_circularity(cnt) < MIN_CONTOUR_CIRCULARITY:
            continue

        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroids.append((cx, cy))
                
    return centroids

def process_frame(frame, x_off=0, y_off=0, rad_ratio=0.45):
    img_masked = _apply_circular_mask(frame, x_off, y_off, rad_ratio)
    raw_mask = _isolate_spots(img_masked)
    clean_mask = _clean_noise_morphology(raw_mask)
    return _extract_centroids(frame, clean_mask)

def _get_camera():
    if Picamera2 is None:
        raise RuntimeError("Picamera2 is unavailable. Camera capture must run on the Raspberry Pi environment.")

    global _camera
    if _camera is None:
        pc = Picamera2()
        pc.configure(pc.create_still_configuration())   # default processed RGB "main"
        pc.start()
        _camera = pc
    return _camera

def close_camera():
    global _camera
    with _camera_lock:
        pc = _camera
        _camera = None
        if pc is None:
            return

        stop = getattr(pc, "stop", None)
        if stop is not None:
            stop()

        close = getattr(pc, "close", None)
        if close is not None:
            close()


def capture_frame():
    with _camera_lock:
        pc = _get_camera()
        img = pc.capture_array("main")  # HxWx3 RGB uint8
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


atexit.register(close_camera)


if __name__ == "__main__":
    # Ensure you use the correct path to your input image
    input_img = capture_frame()
    if input_img is not None:
        results = process_frame(input_img, MASK_X_OFFSET, MASK_Y_OFFSET, MASK_RADIUS_RATIO)
        print(f"Found {len(results)} foam bits.")
