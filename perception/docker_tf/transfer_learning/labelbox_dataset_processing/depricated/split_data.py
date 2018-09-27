import os
import glob
from PIL import Image
import numpy as np
from sklearn.model_selection import train_test_split

imgs_jpg = glob.glob('Images/*.jpeg')
for img_dir in imgs_jpg:
    print(img_dir[:-5])
    im = Image.open(img_dir)
    im.save(img_dir[:-5] + ".png")
    os.remove(img_dir)


X = sorted(glob.glob('Images/*'))
Y = sorted(glob.glob('Annotations/*'))

X_set = set([os.path.basename(os.path.splitext(x)[0]) for x in X])
Y_set = set([os.path.basename(os.path.splitext(y)[0]) for y in Y])

Z = X_set.symmetric_difference(Y_set)
Z = [z + os.path.splitext(X[0])[1] for z in Z]

for z in Z:
    os.remove('Images/' + z)
X = sorted(glob.glob("Images/*"))

X_train, X_test, Y_train, Y_test = train_test_split(X, Y,
                                                    test_size=0.30, random_state=10)

if not os.path.exists('train'):
    os.makedirs('train')
if not os.path.exists('test'):
    os.makedirs('test')

for image, xml_file in zip(X_train, Y_train):
    os.rename(xml_file, 'train/' + os.path.basename(xml_file))
    os.rename(image, 'train/' + os.path.basename(image))

for image, xml_file in zip(X_test, Y_test):
    os.rename(xml_file, 'test/' + os.path.basename(xml_file))
    os.rename(image, 'test/' + os.path.basename(image))
