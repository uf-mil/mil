import cv2 as cv
import numpy as np
import os

from image_geometry import PinholeCameraModel
from subjugator_msgs.srv import (
    VisionRequest,
    VisionRequest2D,
    VisionRequest2DResponse,
    VisionRequestResponse,
)

file_path = './test_images/dock_blue1.jpg'

if not os.path.exists(file_path):
    print(f"Path does not exist: {file_path}")

image = cv.imread(file_path)

gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

# Apply Gaussian blur to the image
blurred = cv.GaussianBlur(gray, (5, 5), 0)

# Use Canny edge detection
edges = cv.Canny(blurred, 50, 150)

# Find contours in the edged image
contours, _ = cv.findContours(edges, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

# Loop over the contours
for contour in contours:
    # Approximate the contour to a polygon
    epsilon = 0.02 * cv.arcLength(contour, True)
    approx = cv.approxPolyDP(contour, epsilon, True)

    # If the approximated contour has 4 vertices, it's a square (or rectangle)
    if len(approx) == 4:
        x, y, w, h = cv.boundingRect(approx)

        # Calculate the aspect ratio
        aspect_ratio = float(w) / h

        # Check if the aspect ratio is close to 1 (square)
        if 0.95 <= aspect_ratio <= 1.05:
            cv.drawContours(image, [approx], -1, (0, 255, 0), 2)

# Display the result
cv.imshow("Squares Detected", image)
cv.waitKey(0)
cv.destroyAllWindows()