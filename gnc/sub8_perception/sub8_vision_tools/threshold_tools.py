import numpy as np
import cv2
import rospy
from sub8_ros_tools.func_helpers import Cache


def bgr_vec_to_hsv(vector):
    """hsv = bgr_vec_to_hsv(np.array([20, 20, 20]))"""
    deep = np.reshape(vector, (1, 1, 3))
    hsv = cv2.cvtColor(deep.astype(np.float32), cv2.COLOR_BGR2HSV)
    return np.squeeze(hsv)


@Cache
def get_threshold(parameter_basename, prefix='bgr'):
    possible_params = rospy.get_param_names()
    bounds = []

    for param in possible_params:
        if param.startswith(parameter_basename) and (prefix in param):
            param_value = rospy.get_param(param)
            bounds.append(np.array(param_value))

    if len(bounds) < 2:
        raise(KeyError("Could not find parameter starting with {} with prefix {}".format(parameter_basename, prefix)))

    vals = np.vstack(bounds)
    upper_bound = np.max(vals, axis=0)
    lower_bound = np.min(vals, axis=0)
    return np.vstack([lower_bound, upper_bound]).transpose()


def param_thresh(rgb_image, parameter_basename, prefer='bgr'):
    """This assumes a bgr encoding"""
    prefer = prefer.lower()

    if prefer == 'bgr':
        thresh_image = rgb_image
    elif prefer == 'hsv':
        thresh_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    else:
        raise(TypeError("You did not choose bgr or hsv, how am I supposed to handle that?"))

    _range = get_threshold(parameter_basename)
    color_mask = cv2.inRange(thresh_image, _range[:, 0], _range[:, 1])
    return color_mask, _range


def grow_cluster(image, prior_range):
    pass


if __name__ == '__main__':
    from sklearn import cluster  # noqa
    import visual_threshold_tools
    from mayavi import mlab

    img = cv2.imread('/home/sub8/Pictures/Selection_001.png')
    cv2.imshow('ss', img)
    thresh, _ = param_thresh(img, '/color/buoy/red')
    _range = np.array([[0.0, 0.0, 49.572], [56.661, 46.053, 201.8835]])

    seg_image = cv2.inRange(img, _range[0, :], _range[1, :])
    nonseg_image = np.logical_not(seg_image)

    seeds = (np.ravel(seg_image).astype(np.uint8) + 1)[::50]
    top = np.average(_range, axis=0)
    bot = np.array([255, 255, 255]) - top
    # clust = cluster.KMeans(n_clusters=2, init=np.vstack([top, bot]))
    # clust = cluster.AgglomerativeClustering(n_clusters=2)

    in_box = img[seg_image.astype(np.bool)]
    out_box = img[nonseg_image]

    all_list = np.reshape(img, (-1, 3))[::50]
    rgb_list = np.reshape(in_box, (-1, 3))
    # clust = cluster.MeanShift(seeds=seeds)
    clust = cluster.MeanShift()

    print rgb_list.shape

    clust.fit(all_list)

    # print clust.cluster_centers_
    visual_threshold_tools.points_with_labels(
        all_list[:, 0],
        all_list[:, 1],
        all_list[:, 2],
        clust.labels_,
        scale_factor=5.0
    )
    mlab.show()
