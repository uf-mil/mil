import numpy as np
import cv2
from scipy.signal import ricker


def mexh(pts, scale):
    """Compute a 2d Mexican Hat wavelet 'kernel'"""
    mexican_hat_1d = ricker(pts, scale)
    mexican_hat_2d = np.outer(mexican_hat_1d, mexican_hat_1d)
    return mexican_hat_2d


def circle_kernel(scale=15, mag=1):
    """Compute a circular kernel"""
    kernel = np.ones((scale * 2, scale * 2)) * -1 * mag
    midpoint = (scale, scale)
    cv2.circle(kernel, midpoint, midpoint[0] / 2, 1 * mag, -1)

    return kernel


def hog_image(image, window_size=100, bin_n=4):
    """Quick convolutional sliding-window computation of HOG features
    The output is in stacked image form

    >>> hog_image(image, window_size=sz, bin_n=bin_n)

    HOG - histogram of oriented gradients
    """
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


def conv_features(image):
    """Generate features by convolving them with our known kernel bank
    Returns an N channel image
    """
    all_convolutions = []
    avg = np.ones((15, 15)) / (15. * 15.)

    for kernel in useful_kernels:
        # Convolve with kernel
        flt = cv2.filter2D(image.astype(np.float32), -1, kernel)
        # Take an average
        flt = cv2.filter2D(flt, -1, avg)
        # Normalize
        flt = (flt - (2 * np.min(flt))) / (np.max(flt) - np.min(flt))
        all_convolutions.append(flt)

    return np.dstack(all_convolutions)

useful_kernels = [
    circle_kernel(5),
    circle_kernel(15),
    circle_kernel(30),
    # circle_kernel(100),
    circle_kernel(5, mag=-1),
    circle_kernel(15, mag=-1),
    circle_kernel(30, mag=-1),
    # circle_kernel(100, mag=-1),
]

# I know this looks ugly.
ksize = 10
box = np.ones((ksize, ksize))
box[ksize // 2:, :] = -1
useful_kernels.append(np.copy(box))

box = np.ones((ksize, ksize))
box[:ksize // 2, :] = -1
useful_kernels.append(np.copy(box))

box = np.ones((ksize, ksize))
box[:, ksize // 2:] = -1
useful_kernels.append(np.copy(box))

box = np.ones((ksize, ksize))
box[:, :ksize // 2] = -1
useful_kernels.append(np.copy(box))

# box = np.ones((ksize * 2, ksize * 2))
# box[ksize:, :] = -1
# useful_kernels.append(np.copy(box))

# box = np.ones((ksize * 2, ksize * 2))
# box[:ksize, :] = -1
# useful_kernels.append(np.copy(box))

# box = np.ones((ksize * 2, ksize * 2))
# box[:, ksize:] = -1
# useful_kernels.append(np.copy(box))

# useful_kernels.append(mexh(20, 5))
# useful_kernels.append(mexh(20, 10))
# useful_kernels.append(mexh(20, 25))
