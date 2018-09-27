import os
sys.path.append('..')
import sys
import glob
import xml.etree.ElementTree as ET
import pandas as pd
import numpy as np
import labelbox2pascal as lb2pa
from PIL import Image
from sklearn.model_selection import train_test_split


def split_data(image_dir='Images', ann_dir='Annotations'):
    imgs_jpg = glob.glob(image_dir + '/*.jpeg')
    for img_dir in imgs_jpg:
        print(img_dir[:-5])
        im = Image.open(img_dir)
        im.save(img_dir[:-5] + ".png")
        os.remove(img_dir)

    X = sorted(glob.glob(image_dir + '/*'))
    Y = sorted(glob.glob(ann_dir + '/*'))

    X_set = set([os.path.basename(os.path.splitext(x)[0]) for x in X])
    Y_set = set([os.path.basename(os.path.splitext(y)[0]) for y in Y])

    Z = X_set.symmetric_difference(Y_set)
    Z = [z + os.path.splitext(X[0])[1] for z in Z]

    for z in Z:
        os.remove(image_dir + '/' + z)
    X = sorted(glob.glob(image_dir + "/*"))

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


def json_to_pascal(labelled_data='json_files/data.json'):
    print("Downloading images and xml files, please wait...")

    # set labeled_data to the file path of the Labelbox JSON export
    if labelled_data != 'json_files/data.json':
        labelled_data = 'json_files/' + labelled_data

    # set ann_output_dir to the file path of the directory to write Pascal VOC
    # annotation files. The directory must exist.
    if not os.path.exists('Annotations'):
        os.mkdir('Annotations')
    ann_output_dir = './Annotations'

    # set images_output_dir to the file path of the directory to write images.
    # The directory must exist.
    if not os.path.exists('Images'):
        os.mkdir('Images')
    images_output_dir = './Images'

    # call the Labelbox to Pascal conversion
    # NOTE: make sure to specify the correct label_format based on the export
    #  format chosen on Labelbox; 'WKT' or 'XY'.
    lb2pa.from_json(labeled_data=labelled_data, annotations_output_dir=ann_output_dir,
                    images_output_dir=images_output_dir, label_format='XY')

    print('Done getting xml files')


def xml_to_csv(path):
    xml_list = []
    for xml_file in glob.glob(path + '/*.xml'):
        tree = ET.parse(xml_file)
        root = tree.getroot()
        for member in root.findall('object'):
            if(member[4].tag == 'polygon'):
                if(len(member[4]) == 8):

                    x1 = int(member[4][0].text)
                    y1 = int(member[4][1].text)
                    x2 = int(member[4][2].text)
                    y2 = int(member[4][3].text)
                    x3 = int(member[4][4].text)
                    y3 = int(member[4][5].text)
                    x4 = int(member[4][6].text)
                    y4 = int(member[4][7].text)

                    max_x = max(x1, x2, x3, x4)
                    max_y = max(y1, y2, y3, y4)
                    min_x = min(x1, x2, x3, x4)
                    min_y = min(y1, y2, y3, y4)

                    file_name = (root.find('filename').text)
                    if os.path.splitext(file_name)[1] == '.jpeg':
                        file_name = os.path.splitext(file_name)[0] + '.png'

                    value = (file_name,
                             int(root.find('size')[0].text),
                             int(root.find('size')[1].text),
                             member[0].text,
                             min_x,
                             min_y,
                             max_x,
                             max_y
                             )
                else:
                    x1 = int(member[4][0].text)
                    y1 = int(member[4][1].text)
                    x2 = int(member[4][2].text)
                    y2 = int(member[4][3].text)

                    max_x = max(x1, x2)
                    max_y = max(y1, y2)
                    min_x = min(x1, x2)
                    min_y = min(y1, y2)

                    file_name = (root.find('filename').text)
                    if os.path.splitext(file_name)[1] == '.jpeg':
                        file_name = os.path.splitext(file_name)[0] + '.png'

                    value = (file_name,
                             int(root.find('size')[0].text),
                             int(root.find('size')[1].text),
                             member[0].text,
                             min_x,
                             min_y,
                             max_x,
                             max_y
                             )

                xml_list.append(value)

            else:
                file_name = (root.find('filename').text)
                if os.path.splitext(file_name)[1] == '.jpeg':
                    file_name = os.path.splitext(file_name)[0] + '.png'

                value = (file_name,
                         int(root.find('size')[0].text),
                         int(root.find('size')[1].text),
                         member[0].text,
                         int(member[4][0].text),
                         int(member[4][1].text),
                         int(member[4][2].text),
                         int(member[4][3].text)
                         )
                xml_list.append(value)
    column_name = ['filename', 'width', 'height',
                   'class', 'xmin', 'ymin', 'xmax', 'ymax']
    xml_df = pd.DataFrame(xml_list, columns=column_name)
    return xml_df
