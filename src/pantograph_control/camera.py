import cv2
import numpy as np
import math

# --- TUNABLE PARAMETERS ---
MASK_RADIUS_RATIO = 0.33 
MASK_Y_OFFSET = -70
MASK_X_OFFSET = -30
MIN_FOAM_AREA = 10000
FIXED_THRESHOLD_VALUE = 180

def process_frame(frame):
    # STEP 1: Get Red Channel (Best contrast for the laser light)
    # Foam is pink/white (High Red), glass background is dark/purple (Lower Red)
    red = frame[:, :, 2]
    cv2.imwrite('1_red_channel.png', red)

    # STEP 2: Create and apply a simple circular mask
    h, w = red.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    center = (w // 2 + MASK_X_OFFSET, h // 2 + MASK_Y_OFFSET)
    radius = int(min(h, w) * MASK_RADIUS_RATIO)
    cv2.circle(mask, center, radius, 255, -1)
    cv2.imwrite('2_mask_shape.png', mask)
    
    masked_red = cv2.bitwise_and(red, red, mask=mask)
    cv2.imwrite('3_masked_output.png', masked_red)

    # STEP 3: Simple Binary Threshold
    # Any pixel > FIXED_THRESHOLD_VALUE becomes 255 (white), else 0 (black)
    _, binary = cv2.threshold(masked_red, FIXED_THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
    cv2.imwrite('4_binary_threshold.png', binary)

    # STEP 4: Find Contours
    # This requires the binary image from Step 3
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    foam_locations = []
    # Create a copy for drawing visual results
    visual_output = frame.copy()

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if MIN_FOAM_AREA < area:
            print(area)
            # Calculate center point
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cradius = int(math.sqrt(area / math.pi))
                foam_locations.append((cx, cy))
                # Draw a circle on the final debug image
                cv2.circle(visual_output, (cx, cy), cradius, (0, 255, 0), 3)

    cv2.imwrite('5_final_detection.png', visual_output)
    return foam_locations

if __name__ == "__main__":
    # Ensure you use the correct path to your input image
    input_img = cv2.imread(r'/home/dmalexa5/pantograph_ws/pantograph_automation/pre_filter.png')
    if input_img is not None:
        results = process_frame(input_img)
        print(f"Found {len(results)} foam bits.")