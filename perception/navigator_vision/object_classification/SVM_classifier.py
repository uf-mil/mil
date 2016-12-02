from sklearn import svm
import pickle


class SVMClassifier(object):

    def __init__(self):
        self.clf = svm.SVC()
        self.number = 0

    def classify(self, desc):
        self.clf.predict(desc)

    def train(self, desc, clss):
        self.clf.fit(desc, clss)

    def pickle(self, name):
        pickle.dump(self, open(name, "wb"))
