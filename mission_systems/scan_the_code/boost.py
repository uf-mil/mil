import numpy as np
import cv2
from base import Base

face_cascade = cv2.CascadeClassifier('cascade1.xml')

crawl = Base.get_images("/home/tess/bags/stc/david.bag", "/stereo/right/image_raw")
for img in crawl:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.5, 6)
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

    cv2.imshow('img', img)
    cv2.waitKey(33)
cv2.destroyAllWindows()
