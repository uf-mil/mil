import cv2
import os

# change this to the diretory that has all the training images in it
base = os.path.dirname(os.path.abspath(__file__))
folder = base + "/train_vid"
f = open(base + "/roi", 'w')

refPt = []
cropping = False


def click_and_crop(event, x, y, flags, param):
    global refPt, cropping
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt = [(x, y)]
        cropping = True
    elif event == cv2.EVENT_LBUTTONUP:
        refPt.append((x, y))
        cropping = False
        cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
        cv2.imshow("image", image)


for i in os.listdir(os.path.abspath(folder)):
    if i.endswith(".jpg"):
        name = folder + "/" + i
        image = cv2.imread(name)
        clone = image.copy()
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", click_and_crop)
        while True:
            cv2.imshow("image", image)
            key = cv2.waitKey(1) & 0xFF
            # if key == orcd("r"):
            #     image = clone.copy()
            if key == ord("c"):
                break

        if len(refPt) == 2:
            f.write(str(refPt[0][1]) + ", " + str(refPt[1][1]) + ", " + str(refPt[0][0]) + ", " + str(refPt[1][0]) + "\n")
            refPt = []

cv2.destroyAllWindows()
