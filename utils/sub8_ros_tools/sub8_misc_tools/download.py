import urllib2
import os
import zipfile
import cStringIO as StringIO

'''
This file contains utilities for downloading a file from the internet
We're using this because I don't want to track 20MB files in Git.

[1] Extracting zipfiles
    http://stackoverflow.com/questions/9431918

[2] Unzip binary directly
    http://stackoverflow.com/questions/18966672

[3] Download a file via http
    http://stackoverflow.com/questions/22676
'''


def download_and_unzip(url, output_dir):
    '''Download and unzip a file at $url,
        then put the contained files at $output_dir
    '''
    try:
        html = download(url)
    except:
        raise(IOError("Could not load file at {}".format(url)))

    fake_file = StringIO.StringIO(html)

    zip_ = zipfile.ZipFile(fake_file, "r")
    for file_path in zip_.namelist():
        path, file_name = os.path.split(file_path)
        file_like = zip_.open(file_path)

        f = open(os.path.join(output_dir, file_name), 'w')
        f.write(file_like.read())
        f.close()


def download(url, output_filename=None):
    '''Download a file at $url, and return the html
        If you set an output location, it will also write the file
    '''
    response = urllib2.urlopen(url)
    html = response.read()
    if output_filename is not None:
        f = open(output_filename, 'w')
        f.write(html)
        f.close()
    return html


if __name__ == '__main__':
    sub_model_url = "http://goo.gl/f0ennf?gdriveurl"
    download_and_unzip(sub_model_url, '.')