import pickle


class Depickle(object):

    def __init__(self):
        mypickle = pickle.load(open("/home/tess/bags/color_train.py", 'rb'))
        file = open('rois.txt', )
        for m in mypickle.bag_to_rois:
            file.write(m)
            for f in m:
                file.write(f)
