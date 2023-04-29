# AUTOGENERATED! DO NOT EDIT! File to edit: nbs/11_shared.ipynb (unless otherwise specified).

__all__ = [
    "SharedCircArrayBuffer",
    "SharedDataCube",
    "save_shared_datacube",
    "SharedOpenHSI",
    "SharedFlirCamera",
]

# Cell

import ctypes
from ctypes import c_float, c_int32, c_uint32
from functools import reduce
from multiprocessing import Array, Process, Queue
from pathlib import Path
from typing import (
    Callable,
    Dict,
    Generic,
    Iterable,
    List,
    Optional,
    Tuple,
    TypeVar,
    Union,
)

import matplotlib.pyplot as plt
import numpy as np
import xarray as xr
from fastcore.foundation import patch
from fastcore.meta import delegates
from tqdm import tqdm

from .data import *

# Cell


# Cell


class SharedCircArrayBuffer(CircArrayBuffer):
    """Circular FIFO Buffer implementation on multiprocessing.Array. Each put/get is a (n-1)darray."""

    def __init__(
        self,
        size: tuple = (100, 100),
        axis: int = 0,
        c_dtype: type = c_int32,
        show_func: Callable[[np.ndarray], "plot"] = None,
    ):
        """Preallocate a array of `size` and type `c_dtype` and init write/read pointer. `c_dtype` needs to be from ctypes"""

        self.shared_data = Array(c_dtype, reduce(lambda x, y: x * y, size))
        self.data = np.frombuffer(self.shared_data.get_obj(), dtype=c_dtype)
        self.data = self.data.reshape(size)

        self.size = size
        self.axis = axis
        self.write_pos = [
            slice(None, None, None) if i != axis else 0 for i in range(len(size))
        ]
        self.read_pos = self.write_pos.copy()
        self.slots_left = self.size[self.axis]
        self.show_func = show_func


@delegates()
class SharedDataCube(CameraProperties):
    """Facilitates the collection, viewing, and saving of hyperspectral datacubes using
    two `SharedCircArrayBuffer`s that swap when save is called."""

    def __init__(self, n_lines: int = 16, processing_lvl: int = -1, **kwargs):
        """Preallocate array buffers"""
        self.n_lines = n_lines
        self.proc_lvl = processing_lvl
        super().__init__(**kwargs)
        self.set_processing_lvl(processing_lvl)
        self.dc_shape = (self.dc_shape[0], self.n_lines, self.dc_shape[1])
        self.dtype_out = c_int32 if self.dtype_out is np.int32 else self.dtype_out
        self.dtype_out = c_float if self.dtype_out is np.float32 else self.dtype_out

        # Only one set of buffers can be used at a time
        self.timestamps_swaps = [DateTimeBuffer(n_lines), DateTimeBuffer(n_lines)]
        self.dc_swaps = [
            SharedCircArrayBuffer(size=self.dc_shape, axis=1, c_dtype=self.dtype_out),
            SharedCircArrayBuffer(size=self.dc_shape, axis=1, c_dtype=self.dtype_out),
        ]
        print(
            f"Allocated {2*4*reduce(lambda x,y: x*y, self.dc_shape)/2**20:.02f} MB of RAM."
        )

        self.current_swap = 0
        self.timestamps = self.timestamps_swaps[self.current_swap]
        self.dc = self.dc_swaps[self.current_swap]

    def __repr__(self):
        return (
            f"DataCube: shape = {self.dc_shape}, Processing level = {self.proc_lvl}\n"
        )

    def put(self, x: np.ndarray):
        """Applies the composed transforms and writes the 2D array into the data cube. Stores a timestamp for each push."""
        self.timestamps.update()
        self.dc.put(self.pipeline(x))


@patch
def save(
    self: SharedDataCube,
    save_dir: str,
    preconfig_meta_path: str = None,
    prefix: str = "",
    suffix: str = "",
) -> Process:
    """Saves to a NetCDF file (and RGB representation) to directory dir_path in folder given by date with file name given by UTC time.
    Save is done in a separate multiprocess.Process."""
    if preconfig_meta_path is not None:
        with open(preconfig_meta_path) as json_file:
            attrs = json.load(json_file)
    else:
        attrs = {}

    self.directory = Path(
        f"{save_dir}/{self.timestamps[0].strftime('%Y_%m_%d')}/"
    ).mkdir(parents=True, exist_ok=True)
    self.directory = f"{save_dir}/{self.timestamps[0].strftime('%Y_%m_%d')}"

    wavelengths = (
        self.binned_wavelengths
        if hasattr(self, "binned_wavelengths")
        else np.arange(self.dc.data.shape[2])
    )

    if hasattr(self, "cam_temperatures"):
        self.coords = dict(
            wavelength=(["wavelength"], wavelengths),
            x=(["x"], np.arange(self.dc.data.shape[0])),
            y=(["y"], np.arange(self.dc.data.shape[1])),
            time=(["time"], self.timestamps.data.astype(np.datetime64)),
            temperature=(["temperature"], self.cam_temperatures.data),
        )
    else:
        self.coords = dict(
            wavelength=(["wavelength"], wavelengths),
            x=(["x"], np.arange(self.dc.data.shape[0])),
            y=(["y"], np.arange(self.dc.data.shape[1])),
            time=(["time"], self.timestamps.data.astype(np.datetime64)),
        )

    fname = f"{self.directory}/{prefix}{self.timestamps[0].strftime('%Y_%m_%d-%H_%M_%S')}{suffix}"

    p = Process(
        target=save_shared_datacube,
        args=(
            fname,
            self.dc.shared_data,
            self.dtype_out,
            self.dc.size,
            self.coords,
            attrs,
            self.proc_lvl,
        ),
    )
    p.start()
    print(f"Saving {fname} in another process.")

    self.current_swap = 0 if self.current_swap == 1 else 1
    self.timestamps = self.timestamps_swaps[self.current_swap]
    self.dc = self.dc_swaps[self.current_swap]
    if hasattr(self, "cam_temperatures"):
        self.cam_temperatures = self.cam_temps_swaps[self.current_swap]
    return p


@patch
def show(
    self: SharedDataCube,
    plot_lib: str = "bokeh",  # Plotting backend. This can be 'bokeh' or 'matplotlib'
    red_nm: float = 640.0,  # Wavelength in nm to use as the red
    green_nm: float = 550.0,  # Wavelength in nm to use as the green
    blue_nm: float = 470.0,  # Wavelength in nm to use as the blue
    robust: bool = False,  # Choose to plot using the 2-98% percentile. Robust to outliers
    hist_eq: bool = False,  # Choose to plot using histogram equilisation
    quick_imshow: bool = False,  # Used to skip holoviews and use matplotlib for a static plot
    **plot_kwargs,  # Any other plotting options to be used in your plotting backend
) -> "bokeh or matplotlib plot":
    """Generate an RGB image from chosen RGB wavelengths with histogram equalisation or percentile options.
    The plotting backend can be specified by `plot_lib` and can be "bokeh" or "matplotlib".
    Further customise your plot with `**plot_kwargs`. `quick_imshow` is used for saving figures quickly
    but cannot be used to make interactive plots."""

    rgb = np.zeros((*self.dc.data.shape[:2], 3), dtype=np.float32)
    if hasattr(self, "binned_wavelengths"):
        rgb[..., 0] = self.dc.data[
            :, :, np.argmin(np.abs(self.binned_wavelengths - red_nm))
        ]
        rgb[..., 1] = self.dc.data[
            :, :, np.argmin(np.abs(self.binned_wavelengths - green_nm))
        ]
        rgb[..., 2] = self.dc.data[
            :, :, np.argmin(np.abs(self.binned_wavelengths - blue_nm))
        ]
    else:
        rgb[..., 0] = self.dc.data[:, :, int(self.dc.data.shape[2] / 2)]
        rgb[..., 1] = self.dc.data[:, :, int(self.dc.data.shape[2] / 2)]
        rgb[..., 2] = self.dc.data[:, :, int(self.dc.data.shape[2] / 2)]

    if robust and not hist_eq:  # scale everything to the 2% and 98% percentile
        vmax = np.nanpercentile(rgb, 98)
        vmin = np.nanpercentile(rgb, 2)
        rgb = ((rgb.astype("f8") - vmin) / (vmax - vmin)).astype("f4")
        rgb = np.minimum(np.maximum(rgb, 0), 1)
    elif hist_eq and not robust:
        img_hist, bins = np.histogram(rgb.flatten(), 256, density=True)
        cdf = img_hist.cumsum()  # cumulative distribution function
        cdf = 1.0 * cdf / cdf[-1]  # normalize
        img_eq = np.interp(
            rgb.flatten(), bins[:-1], cdf
        )  # find new pixel values from linear interpolation of cdf
        rgb = img_eq.reshape(rgb.shape)
    elif robust and hist_eq:
        warnings.warn(
            "Cannot mix robust with histogram equalisation. No RGB adjustments will be made.",
            stacklevel=2,
        )
        rgb /= np.max(rgb)
    else:
        rgb /= np.max(rgb)

    if quick_imshow:
        fig, ax = plt.subplots(figsize=(12, 3))
        ax.imshow(rgb, aspect="equal")
        ax.set_xlabel("along-track")
        ax.set_ylabel("cross-track")
        return fig

    import holoviews as hv

    hv.extension(plot_lib, logo=False)
    rgb_hv = hv.RGB(
        (
            np.arange(rgb.shape[1]),
            np.arange(rgb.shape[0]),
            rgb[:, :, 0],
            rgb[:, :, 1],
            rgb[:, :, 2],
        )
    )

    if plot_lib == "bokeh":
        return (
            rgb_hv.opts(width=1000, height=250, frame_height=int(rgb.shape[0] // 3))
            .opts(**plot_kwargs)
            .opts(xlabel="along-track", ylabel="cross-track", invert_yaxis=True)
        )
    else:  # plot_lib == "matplotlib"
        return (
            rgb_hv.opts(fig_inches=22)
            .opts(**plot_kwargs)
            .opts(xlabel="along-track", ylabel="cross-track", invert_yaxis=True)
        )


# Cell


def save_shared_datacube(
    fname: str,  # NetCDF4 file name (without .nc)
    shared_array: Array,  # multiprocessing.Array shared array
    c_dtype: type,  # numpy data type
    shape: Tuple,  # datacube numpy shape
    coords_dict: Dict,  # coordinates dictionary
    attrs_dict: Dict,  # metadata dictionary
    proc_lvl: int,  # processing level used
) -> "process object":
    """Saves a NetCDF4 file given all the function parameters. Designed to be used with SharedOpenHSI which allocates a shared array."""

    data = np.frombuffer(shared_array.get_obj(), dtype=c_dtype)
    data = data.reshape(shape)

    nc = xr.Dataset(
        data_vars=dict(datacube=(["wavelength", "x", "y"], np.moveaxis(data, -1, 0))),
        coords=coords_dict,
        attrs=attrs_dict,
    )

    """provide metadata to NetCDF coordinates"""
    nc.x.attrs["long_name"] = "cross-track"
    nc.x.attrs["units"] = "pixels"
    nc.x.attrs["description"] = "cross-track spatial coordinates"
    nc.y.attrs["long_name"] = "along-track"
    nc.y.attrs["units"] = "pixels"
    nc.y.attrs["description"] = "along-track spatial coordinates"
    nc.time.attrs["long_name"] = "along-track"
    nc.time.attrs["description"] = "along-track spatial coordinates"
    nc.wavelength.attrs["long_name"] = "wavelength_nm"
    nc.wavelength.attrs["units"] = "nanometers"
    nc.wavelength.attrs["description"] = "wavelength in nanometers."

    if "temperature" in coords_dict.keys():
        nc.temperature.attrs["long_name"] = "camera temperature"
        nc.temperature.attrs["units"] = "degrees Celsius"
        nc.temperature.attrs[
            "description"
        ] = "temperature of sensor at time of image capture"

    nc.datacube.attrs["long_name"] = "hyperspectral datacube"
    nc.datacube.attrs["units"] = "digital number"
    if proc_lvl in (4, 5, 7):
        nc.datacube.attrs["units"] = "uW/cm^2/sr/nm"
    elif proc_lvl in (6, 8):
        nc.datacube.attrs["units"] = "percentage reflectance"
    nc.datacube.attrs["description"] = "hyperspectral datacube"

    nc.to_netcdf(fname + ".nc")

    # quick save the histogram equalised RGB
    rgb = np.zeros((*shape[:2], 3), dtype=np.float32)
    rgb[..., 0] = data[:, :, np.argmin(np.abs(coords_dict["wavelength"][1] - 640.0))]
    rgb[..., 1] = data[:, :, np.argmin(np.abs(coords_dict["wavelength"][1] - 550.0))]
    rgb[..., 2] = data[:, :, np.argmin(np.abs(coords_dict["wavelength"][1] - 470.0))]
    img_hist, bins = np.histogram(rgb.flatten(), 256, density=True)
    cdf = img_hist.cumsum()  # cumulative distribution function
    cdf = 1.0 * cdf / cdf[-1]  # normalize
    img_eq = np.interp(
        rgb.flatten(), bins[:-1], cdf
    )  # find new pixel values from linear interpolation of cdf
    rgb = img_eq.reshape(rgb.shape)
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.imshow(rgb, aspect="equal")
    ax.set_xlabel("along-track")
    ax.set_ylabel("cross-track")
    fig.savefig(fname + ".png", bbox_inches="tight", pad_inches=0)


# Cell


@delegates()
class SharedOpenHSI(SharedDataCube):
    """Base Class for the OpenHSI Camera."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        super().set_processing_lvl(self.proc_lvl)
        if callable(getattr(self, "get_temp", None)):
            self.cam_temps_swaps = [
                CircArrayBuffer(size=(self.n_lines,), dtype=np.float32),
                CircArrayBuffer(size=(self.n_lines,), dtype=np.float32),
            ]
            self.cam_temperatures = self.cam_temps_swaps[self.current_swap]

    def __enter__(self):
        return self

    def __close__(self):
        self.stop_cam()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop_cam()

    def collect(self):
        """Collect the hyperspectral datacube."""
        self.start_cam()
        for i in tqdm(range(self.n_lines)):
            self.put(self.get_img())

            if callable(getattr(self, "get_temp", None)):
                self.cam_temperatures.put(self.get_temp())
        # self.stop_cam()

    def avgNimgs(self, n) -> np.ndarray:
        """Take `n` images and find the average"""
        data = np.zeros(tuple(self.settings["resolution"]) + (n,), np.int32)

        self.start_cam()
        for f in range(n):
            data[:, :, f] = self.get_img()
        self.stop_cam()
        return np.mean(data, axis=2)


# Cell


@delegates()
class SharedFlirCamera(SharedOpenHSI):
    """Interface for FLIR camera"""

    def __init__(self, **kwargs):
        """Initialise FLIR camera"""
        super().__init__(**kwargs)

        try:
            from simple_pyspin import Camera
        except ModuleNotFoundError:
            warnings.warn(
                "ModuleNotFoundError: No module named 'PySpin'.", stacklevel=2
            )

        self.flircam = Camera()
        self.flircam.init()
        self.flircam.GainAuto = "Off"
        self.flircam.Gain = 0
        self.flircam.AcquisitionFrameRateAuto = "Off"
        # self.flircam.AcquisitionFrameRateEnabled = True
        self.flircam.AcquisitionFrameRate = int(
            min(1_000 / (self.settings["exposure_ms"] + 1), 120)
        )

        self.flircam.ExposureAuto = "Off"
        self.flircam.ExposureTime = self.settings["exposure_ms"] * 1e3  # convert to us
        self.flircam.GammaEnabled = False

        self.flircam.Width = (
            self.flircam.SensorWidth
            if self.settings["win_resolution"][1] == 0
            else self.settings["win_resolution"][1]
        )
        self.flircam.Height = (
            self.flircam.SensorHeight
            if self.settings["win_resolution"][0] == 0
            else self.settings["win_resolution"][0]
        )
        self.flircam.OffsetY, self.flircam.OffsetX = self.settings["win_offset"]

    def start_cam(self):
        self.flircam.start()

    def stop_cam(self):
        self.flircam.stop()

    def __close__(self):
        self.flircam.close()

    def get_img(self) -> np.ndarray:
        return self.flircam.get_array()

    def get_temp(self) -> float:
        return self.flircam.DeviceTemperature

    def set_exposure(self, exposure_ms: float):
        """sets the FLIR camera exposure time to `exposure_ms`."""
        self.settings["exposure_ms"] = exposure_ms

        self.flircam.AcquisitionFrameRateAuto = "Off"
        # self.flircam.AcquisitionFrameRateEnabled = True
        self.flircam.AcquisitionFrameRate = int(
            min(1_000 / (self.settings["exposure_ms"] + 1), 120)
        )
        self.flircam.ExposureAuto = "Off"
        self.flircam.ExposureTime = self.settings["exposure_ms"] * 1e3  # convert to us
