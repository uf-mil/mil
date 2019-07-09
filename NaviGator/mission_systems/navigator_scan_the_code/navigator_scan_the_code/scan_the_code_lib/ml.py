import pickle


class Config():

    def __init__(self):
        self.mymap = {'r': 1, 'b': 2, 'y': 3, 'k': 4}
        self.inv_map = {v: k for k, v in self.mymap.iteritems()}

    def get_class(self, val):
        return self.inv_map[val]

    def get_val(self, clss):
        return self.mymap[clss]


class Classify(object):

    def __init__(self):
        self.p = pickle.load(open("svm_train.p", 'rb'))

    def get_color(self, color):
        pass


if __name__ == "__main__":
    c = Classify()
