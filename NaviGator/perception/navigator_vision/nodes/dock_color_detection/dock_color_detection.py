import cv2 as cv
import numpy as np


def find_center_pixel(image_path):
    image = cv.imread(image_path)

    gray_image = cv.cvtColor(image, cv.COLOR_RGB2GRAY)

    median = cv.medianBlur(gray_image, 5)

    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])

    sharpened_image = cv.filter2D(median, -1, kernel)

    _, binary = cv.threshold(sharpened_image, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    morphed = cv.morphologyEx(binary, cv.MORPH_CLOSE, kernel)
    morphed = cv.morphologyEx(morphed, cv.MORPH_OPEN, kernel)

    cv.imshow("Morphed", morphed)
    cv.waitKey(0)
    cv.destroyAllWindows()


def red_blue_or_green(pixel):
    red, green, blue = pixel

    max_color = max(red, green, blue)

    if max_color == red:
        return "red"
    elif max_color == green:
        return "green"
    elif max_color == blue:
        return "blue"
    else:
        return "unknown"


find_center_pixel("./test_images/dock_red1.jpg")
