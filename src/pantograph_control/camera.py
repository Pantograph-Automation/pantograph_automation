import cv2
import numpy as np
import math

# --- TUNABLE PARAMETERS ---
MASK_RADIUS_RATIO = 0.37 
MASK_Y_OFFSET = -50
MASK_X_OFFSET = -50

import cv2
import numpy as np
import cv2
import numpy as np
import math
def apply_circular_mask(image, x_offset=0, y_offset=0, radius_ratio=0.45):
    h, w = image.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    center = (w // 2 + x_offset, h // 2 + y_offset)
    radius = int(min(h, w) * radius_ratio)
    cv2.circle(mask, center, radius, 255, -1)
    
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    cv2.imwrite('1_masked_output.png', masked_image)
    return masked_image

def isolate_pink_spots(masked_image):
    """
    Uses Saturation and Value to find bright, colorful spots.
    """
    hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
    
    # We target HIGH Saturation (the 'pinkness') and HIGH Value (the 'brightness')
    # This is usually more stable than Hue for laser-lit foam.
    lower_bound = np.array([0, 170, 120]) 
    upper_bound = np.array([180, 255, 255])
    
    color_mask = cv2.inRange(hsv, lower_bound, upper_bound)
    cv2.imwrite('2_raw_color_mask.png', color_mask)
    return color_mask

def clean_noise_morphology(color_mask):
    # Use a slightly larger kernel to bridge gaps in foam texture
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    
    # Remove noise, then close holes
    opened = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    cleaned_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=2)
    
    cv2.imwrite('3_morphology_cleaned_mask.png', cleaned_mask)
    return cleaned_mask

def extract_centroids(original_image, cleaned_mask, min_area=9000, max_area=100000):
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    centroids = []
    visual_output = original_image.copy()
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area < area < max_area:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroids.append((cx, cy))
                
                # Visual debug
                r = int(math.sqrt(area / math.pi))
                cv2.circle(visual_output, (cx, cy), r, (0, 255, 0), 2)
                cv2.circle(visual_output, (cx, cy), 2, (0, 0, 255), -1)

    cv2.imwrite('4_final_detection.png', visual_output)
    return centroids

def process_frame(frame, x_off=0, y_off=0, rad_ratio=0.45):
    img_masked = apply_circular_mask(frame, x_off, y_off, rad_ratio)
    raw_mask = isolate_pink_spots(img_masked)
    clean_mask = clean_noise_morphology(raw_mask)
    return extract_centroids(frame, clean_mask)

if __name__ == "__main__":
    # Ensure you use the correct path to your input image
    input_img = cv2.imread(r'/home/dmalexa5/pantograph_ws/pantograph_automation/pre_filter.png')
    if input_img is not None:
        results = process_frame(input_img, MASK_X_OFFSET, MASK_Y_OFFSET, MASK_RADIUS_RATIO)
        print(f"Found {len(results)} foam bits.")