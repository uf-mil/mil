#!/usr/bin/python
import argparse
import cv2
import pickle
import numpy as np
import sys
from boost_auto import observe
import sub8_ros_tools

if __name__ == '__main__':

    usage_msg = ("Pass the path to a bag, the start of an image sequence, or 'video' for a webcam to play through the media and look for the learned object.")
    desc_msg = "glhf"

    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(dest='file_name',
                        help="Pass the path to a bag, the start of an image sequence, or 'video' for a webcam.")
    parser.add_argument(dest='classifer', type=str, help="Name of the classifer to use.")
    parser.add_argument('--topic', type=str, help="Name of the topic to use in a bag or the usb camera number.")
    
    args = parser.parse_args(sys.argv[1:])

    clf = cv2.Boost()
    clf.load(args.classifer)

    file_name = args.file_name
    if file_name.split('.')[-1] == 'bag':
        import sub8_vision_tools.labelling.bag_crawler as b
        print "Loading bag..."
        bc = b.BagCrawler(file_name)
        print bc.image_topics[0]
    else:
        import sub8_vision_tools.labelling.image_crawler        
        if file_name == 'video':
            bc = image_crawler.VideoCrawler(file_name, args.topic)
        else:
            bc = image_crawler.ImageCrawler(file_name)

    last_mask = None
    if args.topic is not None:
        assert args.topic in bc.image_topics, "{} not in the bag".format(args.topic)
        print 'Crawling topic {}'.format(args.topic)
        crawl = bc.crawl(topic=args.topic)
    else:
        crawl = bc.crawl(topic=bc.image_topics[0])

    for image in crawl:
        some_observations = observe(image)
        mask = [x for x in [clf.predict(obs) for obs in some_observations]]
        cv2.imshow('image', image)

        segmentation_image = np.reshape(mask, image[:, :, 2].shape)
        # cv2.imshow('im2', image)
        cv2.imshow('segment', (segmentation_image * 250).astype(np.uint8))
        cv2.waitKey(1)

    print 'saving output'
    pickle.dump(data, open(args.output, 'wb'))
