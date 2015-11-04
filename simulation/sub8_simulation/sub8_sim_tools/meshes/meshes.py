from vispy import io
import os
from time import time
import numpy as np
from sub8_misc_tools import download_and_unzip


def load_and_cache_mesh(file_path, url=None):
    '''Load and save a binary-cached version of a mesh'''
    vertices, faces, normals, texcoords = io.read_mesh(file_path)
    if url is None:
        np.savez(file_path, vertices=vertices, faces=faces, normals=normals)
    else:
        np.savez(file_path, vertices=vertices, faces=faces, normals=normals, url=url)
    return vertices, faces, normals, texcoords


def load_from_cache(path):
    array_data = np.load(path + '.npz')
    vertices = array_data['vertices']
    faces = array_data['faces']
    normals = array_data['normals']
    texcoords = None
    return vertices, faces, normals, texcoords


def load_mesh(path):
    dir_, file_ = os.path.split(path)
    urls = {
        'Sub.obj': "http://goo.gl/f0ennf?gdriveurl"
    }

    # Is this a file we downloaded?
    if file_ in urls.keys():
        if os.path.exists(path + '.npz'):
            # Check if the file is out of date
            array_data = np.load(path + '.npz')

            # Are we out of date?
            if 'url' not in array_data.keys():
                print 'SIM: Existing model does not contain a version for {}, downloading a new one...'.format(file_)
                download_and_unzip(urls[file_], dir_)
                print urls[file_]
                return load_and_cache_mesh(path, url=urls[file_])

            elif array_data['url'] == urls[file_]:
                # No - just load the cache
                return array_data['vertices'], array_data['faces'], array_data['normals'], None

            # Yes - download a new copy
            else:
                print 'SIM: Existing model out of date for {}, downloading a new one...'.format(file_)
                download_and_unzip(urls[file_], dir_)
                return load_and_cache_mesh(path, url=urls[file_])

        else:
            # If we don't have the file cached, there's no way we've downloaded it
            print 'SIM: Model for {} does not exist, downloading it...'.format(file_)
            download_and_unzip(urls[file_], dir_)
            return load_and_cache_mesh(path, url=urls[file_])

    # It's not a file we downloaded, load it normally
    elif os.path.exists(path + '.npz'):
        return load_from_cache(path)

    elif os.path.exists(path):
        print 'SIM: Loading and then cacheing {}'.format(path)
        return load_and_cache_mesh(path)

    else:
        raise Exception('SIM ERROR: Something went wrong, we could not load a mesh at {}'.format(file_))


_filepath = os.path.dirname(os.path.realpath(__file__))

tic = time()
print "SIM: Loading Transdec model"
Transdec = load_mesh(os.path.join(_filepath, 'transdec.obj'))
print "SIM: Loading Sub8 reduced mesh model"
Sub8 = load_mesh(os.path.join(_filepath, 'Sub.obj'))
tic = time() - tic

print "SIM: All meshes loaded, took {} seconds".format(tic)