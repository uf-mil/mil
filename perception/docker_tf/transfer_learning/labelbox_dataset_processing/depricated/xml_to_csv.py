import os
import glob
import pandas as pd
import xml.etree.ElementTree as ET


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


def main():
    for folder in ['train', 'test']:
        #image_path = os.path.join(os.getcwd(), ('images/' + folder))
        xml_df = xml_to_csv(folder)
        xml_df.to_csv((folder + '_labels.csv'), index=None)
        #xml_df.to_csv(('images/' + folder + '_labels.csv'), index=None)
        print('Successfully converted xml to csv.')

if __name__ == '__main__':
    main()
