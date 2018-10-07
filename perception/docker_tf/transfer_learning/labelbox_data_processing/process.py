import os
import glob
import pandas as pd
import xml.etree.ElementTree as ET
from __init__ import xml_to_csv, json_to_pascal, split_data


def main():
    done = False
    while(not done):
        print("1. JSON file in XY format")
        print("2. PASCALVOC XML and PNG pairs \n")
        data = input("Enter the number for type of data input. \n")
        if data == 1 or data == 2:
            done = True
    '''
    TODO: This will eventually load a config for one of the three projects
    from their respective repos and offer a choice of pre-made labelmaps to
    train off of.
    '''
    done = False
    while(not done):
        print("1. NaviGator")
        print("2. SubjuGator [Easy Processing Not Implemented]")
        print("3. PropaGator [Easy Processing Not Implemented] \n")
        labelmap = input(
            "Enter the number for the project you wish to train \n")
        if labelmap == 1 or labelmap == 2 or labelmap == 3:
            done = True

    '''
    At this point we load the config file from the respective repo and loop through the options and make the user enter which one they want to use.
    '''

    if data == 1:
        json_name = raw_input(
            "Please enter the full name of the json file you added. Example: data.json \n")
        json_to_pascal(json_name)
        split_data()

    elif data == 2:
        print("Place the folder containing the images and annotations into the labelbox_data_processing directory.")
        image_dir = raw_input(
            "Enter the name of the directory containing the images (It can be the same as the one containing the annotations). Example: Images \n")
        ann_dir = raw_input(
            "Enter the name of the directory containing the annotations (It can be the same as the one containing the images). Example: Annotations \n ")
        split_data(image_dir, ann_dir)

    for folder in ['train', 'test']:
        #image_path = os.path.join(os.getcwd(), ('images/' + folder))
        xml_df = xml_to_csv(folder)
        xml_df.to_csv((folder + '_labels.csv'), index=None)
        #xml_df.to_csv(('images/' + folder + '_labels.csv'), index=None)
        print('Successfully converted xml to csv.')

if __name__ == '__main__':
    main()
