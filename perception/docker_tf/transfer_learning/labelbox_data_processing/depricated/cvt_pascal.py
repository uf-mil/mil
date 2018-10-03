import os
import sys
sys.path.append('..')
# import labelbox2pascal library
import labelbox2pascal as lb2pa
print("Downloading images and xml files, please wait...")

# set labeled_data to the file path of the Labelbox JSON export
labeled_data = 'json_files/data.json'

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
lb2pa.from_json(labeled_data=labeled_data, annotations_output_dir=ann_output_dir,
                images_output_dir=images_output_dir, label_format='XY')

print('Done getting xml files')
