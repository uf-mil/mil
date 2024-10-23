from dockdeliver_pipeline import GripPipeline
import cv2 as cv
import numpy as np
import math


img_path = "dock2-blue.png"
image = cv.imread(img_path)

pipeline = GripPipeline()

pipeline.process(image)
img = pipeline.hsv_threshold_output
img2 = pipeline.blur_output
contours = pipeline.filter_contours_output


#print_image_values(img)
print(contours[0])
centroid = np.mean(contours[0], axis=0)
print(centroid[0])
print((centroid[0][0],centroid[0][1]))

cv.imshow("blur", img2)
cv.imshow("threshold", img)
img3 = cv.drawContours(img2, contours, -1, (0,255,0), 3)
img3 = cv.circle(img3, ((math.ceil(centroid[0][0]),math.ceil(centroid[0][1]))), radius=2, color=(0, 0, 255), thickness=1)
cv.imshow("contours", img3)



cv.waitKey(0)
cv.destroyAllWindows()