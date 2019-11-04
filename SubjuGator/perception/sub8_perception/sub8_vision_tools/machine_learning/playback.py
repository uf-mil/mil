import argparse
import cv2
import pickle
import numpy as np
import sys
from adaboost import observe
from mil_ros_tools import BagCrawler
from sub8_vision_tools import ImageCrawler, VideoCrawler


if __name__ == '__main__':
    usage_msg = "Pass the path to a bag, the start of an image sequence,or 'video' \
                 for a webcam to play through the media and look for the learned object."
    desc_msg = "glhf"

    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(dest='file_name',
                        help="Pass the path to a bag, the start of an image sequence, or 'video' for a webcam.")
    parser.add_argument(
        dest='classifer',
        type=str,
        help="Name of the classifer to use. Right now only sklearn classifers.")
    parser.add_argument('--topic', type=str, help="Name of the topic to use in a bag or the usb camera number.")

    args = parser.parse_args(sys.argv[1:])

    clf = pickle.load(open(args.classifer, "rb"))

    file_name = args.file_name
    if file_name.split('.')[-1] == 'bag':
        bc = BagCrawler(file_name)
        print bc.image_topics[0]
    else:
        if file_name == 'video':
            bc = VideoCrawler(file_name, args.topic)
        else:
            bc = ImageCrawler(file_name)

    last_mask = None
    if args.topic is not None:
        assert args.topic in bc.image_topics, "{} not in the bag".format(args.topic)
        print 'Crawling topic {}'.format(args.topic)
        crawl = bc.crawl(topic=args.topic)
    else:
        crawl = bc.crawl(topic=bc.image_topics[0])

    for image in crawl:
        some_observations = observe(image)
        mask = clf.predict(some_observations.reshape(-1, some_observations.shape[1])) * 255
        cv2.imshow('image', image)
        cv2.imshow('mask', mask.reshape(image.shape[:2]).astype(np.uint8))
        cv2.waitKey(1)

    print 'saving output'
    '''
    It was unclear what this is supposed to do when ensuring it complies with pep8.
    data is undefined.

    pickle.dump(data, open(args.output, 'wb'))
    '''
