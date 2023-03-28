from typing import Optional

import numpy as np
import scipy
from scipy.signal import fftconvolve


class StreamedBandpass:
    """
    Class for bandpass filtering sequential batches of data. Takes care of the
        overlap-adding that needs to be done when filtering sequential batches
        of data individually.

    Attributes:
        lower (float): The lower bound of the desired frequency.
        upper (float): The upper bound of the desired frequency.
        trans_width (float): A magical special filter property!
        order (float): Filter order for the bandpass filter. Higher is better, but
            slower to compute.
        rate (Optional[float]): The rate at which data is sent.
        h (Optional[np.ndarray]): The Remez exchange algorithm, if defined. Constructed
            when the filter is made.
    """

    def __init__(
        self,
        lower: float,
        upper: float,
        trans_width: float,
        order: float,
        rate: Optional[float] = None,
    ):
        """
        Constructs the class using the variables provided. Checks if the class is
        ready to make a filter, and if so, the filter is constructed.

        Args:
            lower (float): The lower bound of the desired frequency.
            upper (float): The upper bound of the desired frequency.
            trans_width (float): A magical special filter property!
            order (float): Filter order for the bandpass filter. Higher is better, but
                slower to compute.
            rate (Optional[float]): The rate at which data is sent.
        """
        self.from_prev = None
        self.size = order
        self.h = None
        self.lower = lower
        self.upper = upper
        self.trans_width = trans_width
        self.rate = rate

        if self.is_ready_to_make_filter():
            self.make_filter()

    def is_ready_to_make_filter(self) -> bool:
        """
        Checks whether the filter is ready to be made. Checks that the :attr:`.lower`
        variable is not ``None``.

        Returns:
            bool: Whether or not the filter can be made.
        """
        return (
            self.lower is not None
            and self.upper is not None
            and self.trans_width is not None
            and self.rate is not None
            and self.size is not None
        )

    def make_filter(self) -> None:
        """
        Makes the data filter.

        Raises:
            Exception: Some necessary parameters of the class are ``None``.
        """
        if not self.is_ready_to_make_filter():
            raise Exception(
                "Some parameters have not been filled in yet: ",
                self.__dict__,
            )
        self.h = scipy.signal.remez(
            self.size,
            [
                0,
                self.lower - self.trans_width,
                self.lower,
                self.upper,
                self.upper + self.trans_width,
                self.rate / 2,
            ],
            [0, 1, 0],
            Hz=self.rate,
        )

    def convolve(self, data: np.ndarray) -> np.ndarray:
        """
        Does the filtering.

        Args:
            data (np.ndarray): Numpy array of shape ``(samples, channels)``.

        Returns:
            np.ndarray: The resulting filtered data.
        """
        filtered_data = np.apply_along_axis(
            lambda x: fftconvolve(x, self.h, "full"),
            0,
            data,
        )
        if self.from_prev is not None:
            filtered_data[: self.size - 1] += self.from_prev
        self.from_prev = filtered_data[-(self.size - 1) :]

        return filtered_data[: -(self.size - 1)]
