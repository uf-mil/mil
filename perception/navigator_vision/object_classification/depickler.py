import pickle
import sys
import SVM_classifier


def depicklify(filename):
    sys.modules['SVM_classifier'] = SVM_classifier
    cl = pickle.load(open(filename, 'rb'))
    del sys.modules['SVM_classifier']
    return cl
