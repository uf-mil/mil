import cv2
import numpy as np
import pickle
# from sklearn.neural_network import MLPClassifier
from sklearn.datasets import make_classification
from sub8_vision_tools.machine_learning import balanced_resample, desample_binary
"""
TODO
MUST:
    - Hard negative mining
    - Store/load method
    - Balanced sampling

SHOULD:
    - Cleaner kernel definition
    - Cleaner kernel exploration
    - Easier tool for generating data

CONSIDER:
    - Use sklearn preprocessing standar scaler
        - Whiten data, yo
"""


def create_samples(n_samples, n_features, n_classes):
    X, Y = make_classification(n_samples=n_samples, n_features=n_features, n_informative=n_features / 2, n_classes=n_classes)
    X = X.astype("float32")
    Y = Y.astype("uint32")
    return X, Y


def ncc_kernel(scale=15, mag=1):
    # kernel = np.ones((scale, scale)) * -1 * mag
    # midpoint = (scale // 2, scale // 2)
    # cv2.circle(kernel, midpoint, midpoint[0], 1 * mag, -1)
    kernel = np.ones((scale * 2, scale * 2)) * -1 * mag
    midpoint = (scale, scale)
    cv2.circle(kernel, midpoint, midpoint[0] / 2, 1 * mag, -1)

    return kernel


def ncc(image, mean_thresh, scale=15, mag=1):
    '''Compute normalized cross correlation w.r.t a shadowed pillbox fcn

    The expected scale will vary, so we don't cache it
    '''
    kernel = np.ones((scale, scale)) * -1 * mag
    midpoint = (scale // 2, scale // 2)
    cv2.circle(kernel, midpoint, midpoint[0], 1 * mag, -1)

    mean, std_dev = cv2.meanStdDev(image)

    # Check if the scene is brighter than our a priori target
    if mean > mean_thresh:
        kernel = -kernel

    normalized_cross_correlation = cv2.filter2D((image - mean) / std_dev, -1, kernel)
    renormalized = normalized_cross_correlation

    return renormalized


def run_kernels(image, kernels):
    rs = []
    avg = np.ones((10, 10)) / (10. * 10.)

    for kernel in kernels:
        flt = cv2.filter2D(image.astype(np.float32), -1, kernel)
        flt = cv2.filter2D(flt, -1, avg) / np.max(flt)
        # print np.min(flt)
        # print np.max(flt)
        # cv2.imshow('s', (flt - np.min(flt)) / (np.max(flt) - np.min(flt)))
        # cv2.waitKey(0)
        rs.append(flt)

    rs2 = np.dstack(rs)
    return rs2


def hog(image, window_size=100, bin_n=4):
    gx = cv2.Sobel(image, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(image, cv2.CV_32F, 0, 1)
    mag, ang = cv2.cartToPolar(gx, gy)

    _sum = np.ones((window_size, window_size))

    bins = np.int32(bin_n * ang / (2 * np.pi))
    output_im = np.zeros((ang.shape[0], ang.shape[1], bin_n))
    for n in range(bin_n):
        vals = (bins == n).astype(np.float32)
        output_im[:, :, n] = cv2.filter2D(vals, -1, _sum)
        output_im[:, :, n] /= np.max(output_im[:, :, n])

    return output_im


def other_kernels(image):
    h = []
    for sz in (25, 50, 75, 100):
        for bin_n in (4, 8, 12):
            h.append(hog(image, window_size=sz, bin_n=bin_n))
    return np.dstack(h)

good_kernels = [
    ncc_kernel(5),
    ncc_kernel(15),
    ncc_kernel(30),
    ncc_kernel(100),
    ncc_kernel(5, mag=-1),
    ncc_kernel(15, mag=-1),
    ncc_kernel(30, mag=-1),
    ncc_kernel(100, mag=-1),
]

ksize = 25
top_box = np.ones((ksize, ksize))
top_box[ksize // 2:, :] = -1
good_kernels.append(top_box)

bot_box = np.ones((ksize, ksize))
bot_box[:ksize // 2, :] = -1
good_kernels.append(bot_box)

l_box = np.ones((ksize, ksize))
l_box[:, ksize // 2:] = -1
good_kernels.append(l_box)

r_box = np.ones((ksize, ksize))
r_box[:, :ksize // 2] = -1
good_kernels.append(r_box)

box = np.ones((ksize * 2, ksize * 2))
box[ksize:, :] = -1
good_kernels.append(np.copy(box))

box = np.ones((ksize * 2, ksize * 2))
box[:ksize, :] = -1
good_kernels.append(np.copy(box))

box = np.ones((ksize * 2, ksize * 2))
box[:, ksize:] = -1
good_kernels.append(np.copy(box))


def observe(image):
    im_gs = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    observations = np.reshape(image, (-1, 3)).astype(np.float32)

    kernels_run = run_kernels(image[:, :, 2], good_kernels)
    # kernels_run = run_kernels(im_gs, good_kernels)

    kernel_observations = np.reshape(
        kernels_run, (-1, len(good_kernels))
    ).astype(np.float32)

    # other_kernels_run = other_kernels(image[:, :, 2])
    # other_kernel_obs = np.reshape(other_kernels_run, (-1, other_kernels_run.shape[2]))

    all_observations = np.hstack(
        (
            observations,
            kernel_observations,
            # other_kernel_obs,
        )
    ).astype(np.float32)

    return all_observations


def main():
    from sub8_vision_tools import visual_threshold_tools
    from mayavi import mlab

    data = pickle.load(open("segments.p", "rb"))
    observation_list = []
    label_list = []
    for u_image, u_mask in data:
        # image = u_image[::3, ::3, :]
        # mask = u_mask[::3, ::3]
        image = u_image[0:250, 0:250, :]
        mask = u_mask[0:250, 0:250]
        # cv2.imshow('i', image)
        # cv2.waitKey(0)

        im_gs = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Make labels
        targets = np.zeros(mask.shape)
        background = (mask == cv2.GC_PR_BGD) | (mask == cv2.GC_BGD)
        targets[background] = 0
        targets[np.logical_not(background)] = 1

        target_labels = np.reshape(targets, -1, 1).astype(np.int32)

        observations = observe(image)
        observation_list.append(observations)
        label_list.append(target_labels)

    print 'exiting'
    # exit(0)

    all_observations = np.vstack(observation_list)
    all_labels = np.hstack(label_list)

    print all_observations.shape, all_observations.dtype
    print all_labels.shape, all_labels.dtype

    # all_observations, all_labels = balanced_resample(all_observations, all_labels)
    all_observations, all_labels = desample_binary(all_observations, all_labels)

    print all_observations.shape, all_observations.dtype
    print all_labels.shape, all_labels.dtype

    n_trees = 16
    max_depth = 2
    parameters = {
        # "boost_type": cv2.BOOST_REAL,
        "boost_type": cv2.BOOST_GENTLE,
        # "boost_type": cv2.BOOST_DISCRETE,
        "weak_count": n_trees,
        "weight_trim_rate": 0,
        "max_depth": max_depth
    }

    boost = cv2.Boost()
    # observations = np.reshape(image, (-1, 3)).astype(np.float32)

    print 'training'
    boost.train(all_observations, cv2.CV_ROW_SAMPLE, all_labels, params=parameters)
    print 'predicting'

    image = u_image[:500, :500, :]
    mask = u_mask[:500, :500]

    some_observations = observe(image)
    prediction = [int(x) for x in [boost.predict(obs, returnSum=True) for obs in some_observations]]
    prediction2 = [int(x) for x in [boost.predict(obs) for obs in some_observations]]

    print 'displaying'
    prediction_image = np.reshape(prediction, mask.shape)
    prediction_image2 = np.reshape(prediction2, mask.shape)

    import matplotlib.pyplot as plt

    plt.figure(1)
    plt.imshow(prediction_image)
    plt.figure(2)
    plt.imshow(image)
    plt.figure(3)
    plt.imshow(mask)
    plt.figure(4)
    plt.imshow(prediction_image2[:, :, np.newaxis] * image[:, :])
    plt.show()


if __name__ == "__main__":
    main()
