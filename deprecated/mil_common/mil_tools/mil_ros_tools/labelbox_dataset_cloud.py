#!/usr/bin/env python3
import argparse
import json
import os


def create_labelbox_dataset_json(image_directory, url):
    """
    Returns a JSON string which can be used to add a dataset to labelbox.
    The JSON will point to the images in a directory which is also present
    on a public web server.

    Args:
        image_directory: Directory containing 1 or more .png files, ex: /home/bob/my_dataset
        url: The url of the same directory on a webserver, ex: https://ci.mil.ufl.edu/datasets/my_dataset

    Returns:
        A json string in the format needed to create a labelbox dataset.
    """
    images = []
    for filename in os.listdir(image_directory):
        # Image must be png
        base, extension = os.path.splitext(filename)
        if extension != ".png":
            continue
        full_url = os.path.join(url, filename)
        images.append({"externalId": base, "imageUrl": full_url})
    print(json.dumps(images, indent=True))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generates a JSON file for creating a dataset on labelbox.io using\
                     images hosted on another server."
    )
    parser.add_argument(
        "directory", type=str, help="Directory containing the images for the dataset"
    )
    parser.add_argument(
        "url", type=str, default=None, help="URL where this directory can be accessed"
    )
    args = parser.parse_args()
    create_labelbox_dataset_json(args.directory, args.url)
