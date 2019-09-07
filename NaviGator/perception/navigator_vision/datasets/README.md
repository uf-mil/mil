# navigator_vision datasets
This directory contains the configuration used to create the unified [`navigator_computer_vision`](https://app.labelbox.com/projects/cjm82v7349sed0780ftl7pawi/overview) dataset on labelbox.io from bag files. It also contains the latest exported labels for this dataset.

## Contents

### project_labels.json
The latest exported labels from the labelbox.io project. 

### project_config.json
The configuration file for the labelbox.io project, contains the list of all objects that can be labeled.

### project_bags.yaml
The configuration for the extract_bag_images tool in mil_common, used to produce the datasets by extracting images from bag files as specified in this file.


## Adding datasets

> Note: Datasets on labelbox are NONE-MUTABLE. Any new data must be added in a new dataset (the project can have any number of datasets).

1. Add the new bag files to project_bags.yaml under a new dataset. Follow the style and naming conventions established in that file.
1. Run `extract_bag_images` with project_bags.yaml to create a directory of images for the new dataset
1. Add this directory to /var/www/labelme/datasets/navigator-datasets on the CI server
1. Run the `labelbox_dataset_cloud` tool in `mil_common` with `https://ci.mil.ufl.edu/labelme/datasets/navigator-datasets/<your dataset>` for the URL. Pipe this into a new file `<my_dataset>.json`
1. Add the `<my_dataset>.json` dataset to labelbox.io
1. Add `<my_dataset>` to the `navigator_computer_vision` labelbox project
1. Commit the changes to project_bags.yaml and PR it


