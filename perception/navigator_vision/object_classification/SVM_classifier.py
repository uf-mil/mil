from sklearn import svm
import pickle


class SVMClassifier(object):

    def __init__(self):
        self.clf = svm.SVC(probability=True)
        self.number = 0

    def classify(self, desc):
        desc = desc.reshape(1, len(desc))
        probs = self.clf.predict_proba(desc)
        probs = list(probs.flatten())
        p = max(probs)
        i = probs.index(p)
        return i, p

    def train(self, desc, clss):
        self.clf.fit(desc, clss)

    def pickle(self, name):
        pickle.dump(self, open(name, "wb"))
