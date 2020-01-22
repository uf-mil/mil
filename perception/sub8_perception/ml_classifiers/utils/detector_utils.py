# Utilities for object detector.
import rospkg
import numpy as np
import tensorflow as tf
import cv2

from utils import label_map_util

rospack = rospkg.RosPack()
def parse_label_map(target):
    labelmap = rospack.get_path('sub8_perception')+'/datasets/'+ target + '.pbtxt'
    with open(labelmap) as f:
        txt = f.read()
        labels = []
        ids = []
        # print(txt)
        # txt = txt[2, :]
        full_split = [s.strip().split(': ') for s in txt.splitlines()]
        # print(full_split)
        full_split = full_split[1:]
        # try:
        for i in full_split:
            if len(i) < 2:
                continue
            if isinstance(i[1], str):
                if i[1].isdigit():
                    # print(i[1])
                    ids.append(int(i[1]))
                else:
#                    print(i[1].strip("'"))
                    labels.append(i[1].strip("'"))
            else:
               print(
                     "Error, incorrect key located in labelmap. Should be only id or name. Instead found: ", i[1])
        # except:
            # print("It errored! Whomp whomp. ", i)
    dictionary = dict(zip(ids, labels))
    return dictionary

# Load a frozen infrerence graph into memory
def load_inference_graph(target, classes):
    if target == 'depth':
      gfile = tf.gfile
      print("> ===== Loading Depth Model.")
      inference_model = model.Model(is_training=False,
                                batch_size=batch_size,
                                img_height=1080,
                                img_width=1920,
                                seq_length=3,
                                architecture=nets.RESNET,
                                imagenet_norm=True,
                                use_skip=True,
                                joint_encoder=True)
      vars_to_restore = util.get_vars_to_save_and_restore('model-199160')
      saver = tf.train.Saver(vars_to_restore)
      sv = tf.train.Supervisor(logdir='/tmp/', saver=None)
      with sv.managed_session() as sess:
        saver.restore(sess, model_ckpt)
        if not gfile.Exists(output_dir):
          gfile.MakeDirs(output_dir)
          logging.info('Predictions will be saved in %s.', output_dir)  
      detection_graph = tf.Graph()
      print("> ===== Depth Model Loaded.")
      return detection_graph, sess
    else: 
      PATH_TO_LABELS = rospack.get_path('sub8_perception') + '/datasets/' + target + '.pbtxt'
      label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
      categories = label_map_util.convert_label_map_to_categories(
          label_map, max_num_classes=classes, use_display_name=True)
      category_index = label_map_util.create_category_index(categories)

      print("> ====== Loading frozen graph into memory")
      detection_graph = tf.Graph()
      with detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          PATH_TO_CKPT = rospack.get_path('sub8_perception') + '/ml_classifiers/networks/' + target +'/frozen_inference_graph.pb'
          with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
              serialized_graph = fid.read()
              od_graph_def.ParseFromString(serialized_graph)
              tf.import_graph_def(od_graph_def, name='')
          sess = tf.Session(graph=detection_graph)
      print(">  ====== Inference graph loaded.")
      return detection_graph, sess


# Drawing bounding boxes and distances onto image
def draw_box_on_image(num_objects_detect, score_thresh, scores, boxes, classes,
                      im_width, im_height, image_np, target):
    '''asdfa'''
    bbox = []
    if target == 'stake':
      heart = None
      heart_bbox = None
      oval = None
      oval_bbox = None

    for i in range(num_objects_detect):
        if (scores[i] > score_thresh):

            (left, right, top,
             bottom) = (boxes[i][1] * im_width, boxes[i][3] * im_width,
                        boxes[i][0] * im_height, boxes[i][2] * im_height)
            # top left corner of bbox
            p1 = (int(left), int(top))
            # bottom right corner of bbox
            p2 = (int(right), int(bottom))
            dictionary = parse_label_map(target)
            if dictionary[int(classes[i])] == 'oval':
                if oval > scores[i]:
                    continue
                else:
                    oval = scores[i]
                    oval_bbox = [p1, p2]
                    print("Setting Oval to: ", oval_bbox)
            elif dictionary[int(classes[i])] == 'heart':
                if heart > scores[i]:
                    continue
                else:
                    heart = scores[i] 
                    heart_bbox = [p1, p2] 
                    print("Setting Heart to: ", heart_bbox)
            else:
                if dictionary[int(classes[i])] != 'covered_dracula':
                  bbox.append([p1, p2])
                  cv2.rectangle(image_np, p1, p2, (0, 255, 0), 3, 1)
                  cv2.putText(image_np,
                            'confidence: ' + str("{0:.2f}".format(scores[i])) + 'class: ' + dictionary[int(classes[i])],
                            (int(left), int(top) - 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
                else:
                  print("Found a covered_dracula. Nah.")
    if target =='stake' and (heart is not None or oval is not None):
        print("Returning Stake!")
        if heart is not None:
          bbox.append(heart_bbox)
          cv2.rectangle(image_np, (heart_bbox[0][0] - 20, heart_bbox[0][1] - 20), (0, 255, 0), 3, 1)
          cv2.putText(image_np,
            'confidence: ' + str("{0:.2f}".format(heart)) + 'class: Heart',
            heart_bbox[0] - 20, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
          bbox.append([(0, 0), (0,0)])
        if oval is not None:
          bbox.append(oval_bbox)
          cv2.rectangle(image_np, oval_bbox[0], oval_bbox[1], (0, 255, 0), 3, 1)
          cv2.putText(image_np,
            'confidence: ' + str("{0:.2f}".format(oval)) + 'class: Oval',
            (oval_bbox[0][0] - 20, oval_bbox[0][1] - 20), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)
        else:
          bbox.append([(0,0), (0,0)])
    return image_np, bbox

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
    # Each box represents a part of the image where a particular object was
    # detected.
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name(
        'detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Expand dimensions since the model expects images to have shape: [1,
    # None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)

    (boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: image_np_expanded})
#    print("Scores : ", scores)
#    print("\n Boxes: ", boxes)
    return np.squeeze(boxes), np.squeeze(scores), np.squeeze(classes)

