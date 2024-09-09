import cv2 as cv
import numpy as np

# Function that gets color inside bounds
def getColor(r, g, b):
    leniency = 0.9 # smaller value = more lenient
    if r > leniency*(g+b):
        return "Red"
    elif g > leniency*(r+b):
        return "Green"
    elif b > leniency*(r+g):
        return "Blue"
    else:
        return "Other"

image = cv.imread('./test_images/dock_red1.jpg')

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
            # Get region in bounds of contour
            dockBox = image[y:y+h, x:x+w]

            # Get average color
            avgColorRow = np.average(dockBox, axis=0)
            avgColor = np.average(avgColorRow, axis=0)
            avgColor = avgColor.astype(int)

            # Get the RGB values
            b, g, r = avgColor # for some reason colors are in b, g, r
            color = getColor(r, g, b)
            if color != "other":
                cv.drawContours(image, [approx], -1, (0, 255, 0), 2)
                cv.putText(image, color, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


# Display the result
cv.imshow("Squares Detected", image)
cv.waitKey(0)
cv.destroyAllWindows()