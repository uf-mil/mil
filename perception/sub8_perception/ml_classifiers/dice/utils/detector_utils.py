# Utilities for object detector.
import cv2
import rospkg
import numpy as np
import tensorflow as tf

from utils import label_map_util

rospack = rospkg.RosPack()
TRAINED_MODEL_DIR = rospack.get_path('sub8_perception') + '/ml_classifiers/dice'
# Path to frozen detection graph. This is the actual model+weights that is used for the object detection.
PATH_TO_CKPT = TRAINED_MODEL_DIR + '/frozen_graphs/faster_rcnn_243_dice_v2/frozen_inference_graph.pb'
# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = TRAINED_MODEL_DIR + '/frozen_graphs/dice_labelmap.pbtxt'

NUM_CLASSES = 4

# Load label map using utils provided by tensorflow object detection api
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(
    label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


# Load a frozen infrerence graph into memory
def load_inference_graph():

    print("> ====== Loading frozen graph into memory")
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
        sess = tf.Session(graph=detection_graph)
    print(">  ====== Inference graph loaded.")
    return detection_graph, sess


# Drawing bounding boxes and distances onto image
def draw_box_on_image(num_objects_detect, score_thresh, scores, boxes, classes,
                      im_width, im_height, image_np):
    '''asdfa'''

    for i in range(num_objects_detect):
        if (scores[i] > score_thresh):

            (left, right, top,
             bottom) = (boxes[i][1] * im_width, boxes[i][3] * im_width,
                        boxes[i][0] * im_height, boxes[i][2] * im_height)
            p1 = (int(left), int(top))
            p2 = (int(right), int(bottom))

            cv2.rectangle(image_np, p1, p2, (0, 255, 0), 3, 1)

            cv2.putText(image_np,
                        'confidence: ' + str("{0:.2f}".format(scores[i])),
                        (int(left), int(top) - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)


# Show fps value on image.
def draw_text_on_image(fps, image_np):
    cv2.putText(image_np, fps, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (77, 255, 9), 2)


def detect_objects(image_np, detection_graph, sess):
    '''
    Actual detection .. generate scores and bounding boxes given an image
    :param: numpy array of image to be passed into tensorflow graph
    :param: Loaded Tensorflow Graph (use load_inference graph())
    :param: Tensorflow Session from loaded graph
    :return: numpy array
    :return: numpy array
    :return: numpy array
    '''
    # Definite input and output Tensors for detection_graph
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    # Each box represents a part of the image where a particular object was detected.
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name(
        'detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)

    (boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: image_np_expanded})
    return np.squeeze(boxes), np.squeeze(scores), np.squeeze(classes)
