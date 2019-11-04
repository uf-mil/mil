import cv2
import sys
import pandas as pd

'''
This script allows us to visualize the results of the data processing and to ensure that all labels are correct placed.
'''

test_df = pd.read_csv('test_labels.csv')
train_df = pd.read_csv('train_labels.csv')

prev_img = None
for index, row in test_df.iterrows():
    if prev_img != row['filename']:
        img = cv2.imread('test/' + row['filename'])
        prev_img = row['filename']
    cv2.rectangle(img, (row['xmin'], row['ymin']),
                  (row['xmax'], row['ymax']), (0, 0, 255), 5)
    cv2.imshow('image', img)
    cv2.waitKey(0)

for index, row in train_df.iterrows():
    if prev_img != row['filename']:
        img = cv2.imread('train/' + row['filename'])
        prev_img = row['filename']
    cv2.rectangle(img, (row['xmin'], row['ymin']),
                  (row['xmax'], row['ymax']), (0, 0, 255), 5)
    cv2.imshow('image', img)
    k = cv2.waitKey(50)
    print(k)
    if k == 'ESC':
        cv2.destroyAllWindows()
        sys.exit()

cv2.destroyAllWindows()
