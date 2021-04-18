import cv2
import numpy as np


def detect_traffic_light_color(image, bounding_box):
    # Very naive classification:
    # - First convert to HSV for easy color thresholding
    # - Then threshold red, yellow and green hues.
    # - On these masks: detect circles
    # - Then select color which occurs most.

    # Convert image to HSV for easy processing.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Mask for bounding box.
    bb_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.bool)
    bb_mask[
        bounding_box[1] : bounding_box[3],
        bounding_box[0] : bounding_box[2],
    ] = 1.0

    # Color ranges (computed using http://www.speakingsame.com/hsv/index.php)
    # Due to circular nature of hue channel, two masks needed for red.
    red_mask = bb_mask & cv2.inRange(
        hsv, np.array([0, 150, 100]), np.array([10, 255, 255])
    )
    green_mask = bb_mask & cv2.inRange(
        hsv, np.array([40, 50, 50]), np.array([90, 255, 255])
    )
    yellow_mask = bb_mask & cv2.inRange(
        hsv, np.array([15, 100, 100]), np.array([30, 255, 255])
    )

    colors = ["red", "yellow", "green"]
    color = colors[
        np.argmax([np.sum(red_mask), np.sum(yellow_mask), np.sum(green_mask)])
    ]

    return color
