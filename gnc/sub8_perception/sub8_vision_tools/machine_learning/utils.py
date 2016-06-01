import numpy as np
from scipy.stats import mstats
from sklearn import utils


def balanced_resample(data, labels):
    """Do a balanced resampling of data and labels, returning them
    See the test routine at the bottom for an example of behavior
    """
    most_common, num_required = mstats.mode(labels)
    possible_labels = np.unique(labels)

    data_resampled = []
    labels_resampled = []

    for possible_label in possible_labels:
        in_this_label = labels == possible_label

        data_buffered = np.array([])
        data_buffered = np.reshape(data_buffered, (0, data.shape[1]))
        labels_buffered = np.array([])

        while len(data_buffered) < num_required:
            data_buffered = np.vstack([data_buffered, data[in_this_label]])
            labels_buffered = np.hstack([labels_buffered, labels[in_this_label]])

        single_data_resampled, single_labels_resampled = utils.resample(
            data_buffered,
            labels_buffered,
            n_samples=int(num_required),
            replace=True
        )
        data_resampled.append(single_data_resampled)
        labels_resampled.append(single_labels_resampled)

    return np.vstack(data_resampled).astype(data.dtype), np.hstack(labels_resampled).astype(labels.dtype)


def desample_binary(data, labels):
    """Assumes majority is False"""
    need_samples = np.sum(labels == 1)
    in_majority = (labels == 0)
    in_minority = np.logical_not(in_majority)
    majority_data_resampled, majority_labels_resampled = utils.resample(
        data[in_majority],
        labels[in_majority],
        n_samples=min(need_samples * 10, np.sum(labels == 1))
    )

    data_resampled = np.vstack([data[in_minority], majority_data_resampled])
    labels_resampled = np.hstack([labels[in_minority], majority_labels_resampled])

    return data_resampled, labels_resampled


def _pct(data, val):
    """Return the percentage of $data equal to $val"""
    return np.sum(data == val).astype(np.float32) / data.shape[0]

if __name__ == '__main__':
    # todo: make this an actual unittest

    x = np.random.random((100, 3))
    y = (x[:, 0] > 0.25).astype(np.uint8)

    assert not np.isclose(_pct(y, 0), 0.5, atol=0.1)
    assert not np.isclose(_pct(y, 1), 0.5, atol=0.1)

    nx, ny = balanced_resample(x, y)
    print _pct(ny, 0)
    assert np.isclose(_pct(ny, 0), 0.5, atol=0.1)
    print _pct(ny, 1)
    assert np.isclose(_pct(ny, 1), 0.5, atol=0.1)
