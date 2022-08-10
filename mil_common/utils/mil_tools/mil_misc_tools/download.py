"""
This file contains utilities for downloading a file from the internet
We're using this because I don't want to track 20MB files in Git.

[1] Extracting zipfiles
    http://stackoverflow.com/questions/9431918

[2] Unzip binary directly
    http://stackoverflow.com/questions/18966672

[3] Download a file via http
    http://stackoverflow.com/questions/22676
"""
import io as StringIO
import os
import urllib.request
import zipfile
from typing import Optional


def download_and_unzip(url: str, output_dir: str):
    """
    Downloads a zip file at a particular URL and unzips it to a directory.

    Args:
        url (str): The URL to obtain the zip file from.
        output_dir (str): The location of where to write the zip contents to.

    Raises:
        IOError: The file at the URL could not be found/loaded.
    """
    try:
        html = download(url)
    except:
        raise OSError(f"Could not load file at {url}")

    fake_file = StringIO.StringIO(html)

    zip_ = zipfile.ZipFile(fake_file, "r")
    for file_path in zip_.namelist():
        _, file_name = os.path.split(file_path)
        file_like = zip_.open(file_path)

        f = open(os.path.join(output_dir, file_name), "w")
        f.write(file_like.read().decode("utf-8"))
        f.close()


def download(url: str, output_filename: Optional[str] = None) -> str:
    """
    Downloads the contents of a particular URL. If an output filename is also
    specified, the filename is written to with the URL contents.

    Args:
        url (str): The URL to obtain contents from.
        output_filename (str): The filename of the output file to write the
          contents.

    Returns:
        str: The HTML contents of the URL.
    """
    response = urllib.request.urlopen(url)
    html = response.read()
    if output_filename is not None:
        f = open(output_filename, "w")
        f.write(html)
        f.close()
    return html


if __name__ == "__main__":
    sub_model_url = "http://goo.gl/f0ennf?gdriveurl"
    download_and_unzip(sub_model_url, ".")
