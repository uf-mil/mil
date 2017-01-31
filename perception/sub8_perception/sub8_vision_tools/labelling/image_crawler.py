import cv2

class ImageCrawler(object):
    def __init__(self, starting_image):
        '''
        Give it the first image in an image sequence. The sequence should be just abunch of numbered images,
        and the starting image should be the first in that sequence.
        '''
        assert '.' in starting_image, "Please include the first image in your sequence."
        self.image_path = starting_image.rsplit('/', 1)[0] + '/'
        self.starting_image, self.image_format = starting_image.rsplit('/', 1)[1].split('.')


    def crawl(self, topic, max_msgs=float('inf')):
        num_seen = 0
        while num_seen < max_msgs:
            image_number = int(self.starting_image) + int(num_seen)
            image_path = self.image_path + str(image_number) + '.' + self.image_format
            image = cv2.imread(image_path)

            num_seen += 1
            yield image

    @property
    def image_topics(self):
        return [None]

class VideoCrawler(object):
    def __init__(self, source):
        if source is None:
            source = 0
        self.cap = cv2.VideoCapture(source)
        self.cap.read()

    def crawl(self, topic, max_msgs=float('inf')):
        num_seen = 0
        while num_seen < max_msgs:
            num_seen += 1
            yield self.cap.read()[1]

    @property
    def image_topics(self):
        return [None]

if __name__ == '__main__':
    import cv2

    bag = '/Volumes/FLASHDRIVE/2016-05-22-11-31-42_images/0.png'
    bc = ImageCrawler(bag)

    for image in bc.crawl(topic=bc.image_topics[0]):
        print image
        cv2.imshow('current_image', image)
        cv2.waitKey(0)
