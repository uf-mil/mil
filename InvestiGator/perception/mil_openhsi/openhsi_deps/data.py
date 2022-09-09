# AUTOGENERATED! DO NOT EDIT! File to edit: nbs/00_data.ipynb (unless otherwise specified).

__all__ = ['Array', 'Shape', 'CircArrayBuffer', 'CameraProperties', 'DateTimeBuffer', 'DataCube']

# Cell

from fastcore.foundation import patch
from fastcore.meta import delegates
from fastcore.basics import listify
from fastcore.xtras import *
import xarray as xr
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import interp1d
from PIL import Image
from scipy.signal import decimate

from typing import Iterable, Union, Callable, List, TypeVar, Generic, Tuple, Optional
import json
import pickle
from datetime import datetime, timezone, timedelta
from pathlib import Path
import warnings
import pprint

# Cell
#hide

# numpy.ndarray type hints
Shape = TypeVar("Shape"); DType = TypeVar("DType")
class Array(np.ndarray, Generic[Shape, DType]):
    """
    Use this to type-annotate numpy arrays, e.g.
        image: Array['H,W,3', np.uint8]
        xy_points: Array['N,2', float]
        nd_mask: Array['...', bool]
    from: https://stackoverflow.com/questions/35673895/type-hinting-annotation-pep-484-for-numpy-ndarray
    """
    pass

# Cell

class CircArrayBuffer():
    """Circular FIFO Buffer implementation on ndarrays. Each put/get is a (n-1)darray."""

    def __init__(self,
                 size:tuple = (100,100), # Shape of n-dim circular buffer to preallocate
                 axis:int = 0,           # Which axis to traverse when filling the buffer
                 dtype:type = np.int32,  # Buffer numpy data type
                 show_func:Callable[[np.ndarray],"plot"] = None, # Custom plotting function if desired
                ):
        """Preallocate a array of `size` and type `dtype` and init write/read pointer."""
        self.data = np.zeros(size, dtype=dtype)
        self.size = size
        self.axis = axis
        self.write_pos = [slice(None,None,None) if i != axis else 0 for i in range(len(size)) ]
        self.read_pos  = self.write_pos.copy()
        self.slots_left = self.size[self.axis]
        self.show_func = show_func

    def __getitem__(self, key:slice):
        return self.data[key]

    def _inc(self, idx:List[slice]) -> List[slice]:
        """Increment read/write index with wrap around"""
        idx[self.axis] += 1
        if idx[self.axis] == self.size[self.axis]:
            idx[self.axis] = 0
        return idx

    def is_empty(self) -> bool:
        return self.slots_left == self.size[self.axis]

    def put(self, line:np.ndarray):
        """Writes a (n-1)darray into the buffer"""
        self.data[tuple(self.write_pos)] = line

        # if buffer full, update read position to keep track of oldest slot
        self.slots_left -= 1
        if self.slots_left < 0:
            self.slots_left = 0
            self.read_pos = self._inc(self.read_pos)

        self.write_pos = self._inc(self.write_pos)

    def get(self) -> np.ndarray:
        """Reads the oldest (n-1)darray from the buffer"""
        if self.slots_left < self.size[self.axis]:
            val = self.data[tuple(self.read_pos)]
            self.slots_left += 1
            self.read_pos = self._inc(self.read_pos)
            return val
        else:
            return None

    def show(self):
        """Display the data """
        if self.show_func is None:
            if len(self.size) == 2:
                return hv.Image(self.data.copy(), bounds=(0,0,*self.size)).opts(
                    xlabel="wavelength index",ylabel="cross-track",cmap="gray")
            elif len(self.size) == 3:
                # Sum over the last dimensions (assumed wavelength) and show as monochrome
                return hv.Image(np.sum(self.data,axis=-1), bounds=(0,0,*self.size[:2])).opts(
                    xlabel="along-track",ylabel="cross-track",cmap="gray")
            elif len(self.size) == 1:
                print(f"#({self.size[0]}) {self.data}")
        elif self.show_func is not None:
            return self.show_func(self.data)
        else:
            print("Unsupported array shape. Please use 2D or 3D shapes or use your own custom show function")


# Cell

class CameraProperties():
    """Save and load OpenHSI camera settings and calibration"""
    def __init__(self,
                 json_path:str = None,  # Path to settings file
                 pkl_path:str  = None,  # Path to calibration file
                 print_settings:bool = False, # Print out settings file contents
                 **kwargs):
        """Load the settings and calibration files"""
        self.json_path = json_path
        self.pkl_path = pkl_path

        if json_path:
            with open(self.json_path) as json_file:
                self.settings = json.load(json_file)
        else:
            self.settings = {}

        if pkl_path:
            with open(self.pkl_path,'rb') as handle:
                self.calibration = pickle.load(handle)
        else:
            self.calibration = {}

        # overide any settings from settings file with keywords value pairs.
        for key,value in kwargs.items():
            if key in self.settings.keys():
                self.settings[key] = value
                if print_settings:
                    print("Setting File Override: {0} = {1}".format(key, value))
        if print_settings:
            pprint.pprint(self.settings)

    def __repr__(self):
        return "settings = \n" + self.settings.__repr__() + \
               "\n\ncalibration = \n" + self.calibration.__repr__()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        pass

    def dump(self, json_path:str = None, pkl_path:str = None):
        """Save the settings and calibration files"""
        with open(self.json_path[:-5]+"_updated.json" if json_path is None else json_path, 'w') as outfile:
            json.dump(self.settings, outfile,indent=4,)
        with open(self.pkl_path[:-4]+"_updated.pkl" if pkl_path is None else pkl_path,'wb') as handle:
            pickle.dump(self.calibration,handle,protocol=4)


# Cell

@patch
def tfm_setup(self:CameraProperties,
              more_setup:Callable[[CameraProperties],None] = None,
              dtype:Union[np.int32,np.float32] = np.int32,
              lvl:int = 0):
    """Setup for transforms"""
    if self.fast_smile in self.tfm_list:
        self.smiled_size = (np.ptp(self.settings["row_slice"]), self.settings["resolution"][1] - np.max(self.calibration["smile_shifts"]) )
        self.line_buff = CircArrayBuffer(self.smiled_size, axis=0, dtype=dtype)

        # for collapsing spectral pixels into bands
        self.byte_sz = dtype(0).nbytes
        self.width = np.uint16(self.settings["fwhm_nm"]*self.settings["resolution"][1]/np.ptp(self.calibration["wavelengths_linear"]))
        self.bin_rows = np.ptp(self.settings["row_slice"])
        self.bin_cols = self.settings["resolution"][1] - np.max(self.calibration["smile_shifts"])
        self.reduced_shape = (self.bin_rows,self.bin_cols//self.width,self.width)

    if self.fast_bin in self.tfm_list:
        self.binned_wavelengths = self.calibration["wavelengths_linear"].astype(np.float32)
        self.binned_wavelengths = np.lib.stride_tricks.as_strided(self.binned_wavelengths,
                                            strides=(self.width*4,4), # assumed np.float32
                                            shape=(len(self.binned_wavelengths)//self.width,self.width))
        self.binned_wavelengths = np.around(self.binned_wavelengths.mean(axis=1),decimals=1)

    if self.slow_bin in self.tfm_list:
        n_bands = int(np.ptp(self.calibration["wavelengths"])//self.settings["fwhm_nm"])
        # jump by `fwhm_nm` and find closest array index, then let the wavelengths be in the middle between jumps
        self.λs = np.around(np.array([np.min(self.calibration["wavelengths"]) + i*self.settings["fwhm_nm"] for i in range(n_bands+1)]),decimals=1)
        self.bin_idxs = [np.argmin(np.abs(self.calibration["wavelengths"]-λ)) for λ in self.λs]
        self.binned_wavelengths = self.λs[:-1] + self.settings["fwhm_nm"]//2 #
        binned_type = np.float32 if hasattr(self,"need_rad") else dtype
        self.bin_buff = CircArrayBuffer((np.ptp(self.settings["row_slice"]),n_bands), axis=1, dtype=binned_type)

    if self.dn2rad in self.tfm_list:
        # precompute some reference data for converting digital number to radiance
        self.nearest_exposure = self.calibration["rad_ref"].sel(exposure=self.settings["exposure_ms"],method="nearest").exposure

        # use max valid rad_ref luminance if none given.
        if "luminance" not in self.settings.keys():
            self.settings["luminance"] = int(np.max(
                self.calibration["rad_ref"].luminance.where(
                    np.isfinite(
                        self.calibration["rad_ref"]
                        .sel(exposure=self.nearest_exposure)
                        .any(axis=(1, 2))
                    )
                )
            ).data.tolist())

        try:
            dark_radref = self.calibration["rad_ref"].sel(exposure=self.nearest_exposure,luminance=0).isel(luminance=0)
        except (KeyError, ValueError):
            dark_radref = self.calibration["rad_ref"].sel(exposure=self.nearest_exposure,luminance=0)

        self.dark_current = np.squeeze( np.array( self.settings["exposure_ms"]/self.nearest_exposure * dark_radref ) )
        self.ref_luminance = np.squeeze( np.array( self.settings["exposure_ms"]/self.nearest_exposure * \
                             self.calibration["rad_ref"].sel(exposure=self.nearest_exposure,luminance=self.settings["luminance"]) - \
                             self.dark_current ) )
        self.spec_rad_ref = np.float32(self.calibration["sfit"](self.calibration["wavelengths"]))

        self.dark_current = np.float32(self.fast_smile(self.dark_current))
        self.ref_luminance = np.float32(self.fast_smile(self.ref_luminance))

    if hasattr(self,"need_rad_after_fast_bin"):
        self.dark_current = self.fast_bin(self.dark_current)
        self.ref_luminance = self.fast_bin(self.ref_luminance)
        self.spec_rad_ref = np.float32(self.calibration["sfit"]( self.binned_wavelengths ))

    if hasattr(self,"need_rad_after_slow_bin"):
        self.dark_current = self.slow_bin(self.dark_current)
        self.ref_luminance = self.slow_bin(self.ref_luminance)
        self.spec_rad_ref = np.float32(self.calibration["sfit"]( self.binned_wavelengths ))

    if self.rad2ref_6SV in self.tfm_list:
        self.rad_6SV = np.float32(self.calibration["rad_fit"]( self.binned_wavelengths ))

    if more_setup is not None:
        more_setup(self)


# Cell

@patch
def crop(self:CameraProperties, x:np.ndarray) -> np.ndarray:
    """Crops to illuminated area"""
    return x[self.settings["row_slice"][0]:self.settings["row_slice"][1],:]

@patch
def fast_smile(self:CameraProperties, x:np.ndarray) -> np.ndarray:
    """Apply the fast smile correction procedure"""
    for i in range(self.smiled_size[0]):
            self.line_buff.put(x[i,self.calibration["smile_shifts"][i]:self.calibration["smile_shifts"][i]+self.smiled_size[1]])
    return self.line_buff.data


# Cell

@patch
def fast_bin(self:CameraProperties, x:np.ndarray) -> np.ndarray:
    """Changes the view of the datacube so that everything that needs to be binned is in the last axis. The last axis is then binned."""
    buff = np.lib.stride_tricks.as_strided(x, shape=self.reduced_shape,
                        strides=(self.bin_cols*self.byte_sz,self.width*self.byte_sz,self.byte_sz))
    return buff.sum(axis=-1)

@patch
def slow_bin(self:CameraProperties, x:np.ndarray) -> np.ndarray:
    """Bins spectral bands accounting for the slight nonlinearity in the index-wavelength map"""
    for i in range(len(self.bin_idxs)-1):
        self.bin_buff.put( x[:,self.bin_idxs[i]:self.bin_idxs[i+1]].sum(axis=1) )
    return self.bin_buff.data

# Cell

@patch
def dn2rad(self:CameraProperties, x:"Array['λ,x',np.int32]") -> "Array['λ,x',np.float32]":
    """Converts digital numbers to radiance (uW/cm^2/sr/nm). Use after cropping to useable area."""
    #print((self.settings["luminance"] * (x-self.dark_current)/self.ref_luminance*self.spec_rad_ref/self.calibration['spec_rad_ref_luminance']).shape)
    return np.float32( (x - self.dark_current) * self.settings["luminance"]/self.ref_luminance  *  self.spec_rad_ref/self.calibration['spec_rad_ref_luminance'] )

@patch
def rad2ref_6SV(self:CameraProperties, x:"Array['λ,x',np.float32]") -> "Array['λ,x',np.float32]":
    """"""
    # # If wavelength dimension shapes do not match, do some hacks
    # if x.shape[1] < self.rad_6SV.shape[0]:   # use wavelengths after binning to match input
    #     self.rad_6SV = np.float32(self.calibration["rad_fit"](self.binned_wavelengths))
    # elif x.shape[1] < self.rad_6SV.shape[0]: # upsize wavelength range to match input
    #     self.rad_6SV = np.float64(self.calibration["rad_fit"]( np.resize(self.calibration["wavelengths"],x.shape[1]) ))

    return x/self.rad_6SV

# Cell

@patch
def set_processing_lvl(self:CameraProperties, lvl:int = -1, custom_tfms:List[Callable[[np.ndarray],np.ndarray]] = None):
    """Define the output `lvl` of the transform pipeline. Predefined recipies include:
    -1: do not apply any transforms (default),
    0 : raw digital numbers cropped to useable sensor area,
    1 : crop + fast smile,
    2 : crop + fast smile + fast binning,
    3 : crop + fast smile + slow binning,
    4 : crop + fast smile + fast binning + conversion to radiance in units of uW/cm^2/sr/nm,
    5 : crop + fast smile + radiance + fast binning,
    6 : crop + fast smile + fast binning + radiance + reflectance,
    7 : crop + fast smile + radiance + slow binning,
    8 : crop + fast smile + radiance + slow binning + reflectance.
    """
    if   lvl == -1:
        self.tfm_list = []
    elif lvl == 0:
        self.tfm_list = [self.crop]
    elif lvl == 1:
        self.tfm_list = [self.crop,self.fast_smile]
    elif lvl == 2:
        self.tfm_list = [self.crop,self.fast_smile,self.fast_bin]
    elif lvl == 3:
        self.tfm_list = [self.crop,self.fast_smile,self.slow_bin]
    elif lvl == 4:
        self.tfm_list = [self.crop,self.fast_smile,self.fast_bin,self.dn2rad]
        self.need_rad_after_fast_bin = True
    elif lvl == 5:
        self.tfm_list = [self.crop,self.fast_smile,self.dn2rad,self.fast_bin]
    elif lvl == 6:
        self.tfm_list = [self.crop,self.fast_smile,self.fast_bin,self.dn2rad,self.rad2ref_6SV]
        self.need_rad_after_fast_bin = True
    elif lvl == 7:
        self.tfm_list = [self.crop,self.fast_smile,self.dn2rad,self.slow_bin]
    elif lvl == 8:
        self.tfm_list = [self.crop,self.fast_smile,self.dn2rad,self.slow_bin,self.rad2ref_6SV]
    else:
        self.tfm_list = []

    if custom_tfms is not None:
        self.tfm_list = listify(custom_tfms)

    # binning input/output types
    self.dtype_in = np.int32
    self.dtype_out = np.float32 if lvl in (4,5,6,7,8) else np.int32
    if len(self.tfm_list) > 0:
        self.tfm_setup(dtype=self.dtype_in, lvl=lvl)
        self.dc_shape = self.pipeline(self.calibration["flat_field_pic"]).shape
    elif "resolution" in self.settings.keys():
        self.dc_shape = tuple(self.settings["resolution"])
    else:
        self.dc_shape = (1,1) # unused. just for calibration when settings file needs creating

# Cell

@patch
def pipeline(self:CameraProperties, x:np.ndarray) -> np.ndarray:
    """Compose a list of transforms and apply to x."""
    for f in self.tfm_list:
        x = f(x)
    return x


# Cell

class DateTimeBuffer():
    """Records timestamps in UTC time."""
    def __init__(self, n:int = 16):
        """Initialise a nx1 array and write index"""
        self.data = np.arange(n).astype(datetime)
        self.n = n
        self.write_pos = 0

    def __getitem__(self, key:slice) -> datetime:
        return self.data[key]

    def update(self):
        """Stores current UTC time in an internal buffer when this method is called."""
        ts = datetime.timestamp(datetime.now())
        self.data[self.write_pos] = datetime.fromtimestamp(ts, tz=timezone.utc)
        self.write_pos += 1

        # Loop back if buffer is full
        if self.write_pos == self.n:
            self.write_pos = 0


# Cell
from functools import reduce
import psutil

@delegates()
class DataCube(CameraProperties):
    """Facilitates the collection, viewing, and saving of hyperspectral datacubes."""
    def __init__(self,
                 n_lines:int = 16,          # How many along-track pixels desired
                 processing_lvl:int = -1,   # Desired real time processing level
                 warn_mem_use:bool = True,  # Raise error if trying to allocate too much memory (> 80% of available RAM)
                 **kwargs,):
        """Preallocate array buffers"""
        self.n_lines = n_lines
        self.proc_lvl = processing_lvl
        super().__init__(**kwargs)
        self.set_processing_lvl(processing_lvl)

        self.timestamps = DateTimeBuffer(n_lines)
        self.dc_shape = (self.dc_shape[0],self.n_lines,self.dc_shape[1])
        mem_sz = 4*reduce(lambda x,y: x*y, self.dc_shape)/2**20 # MB
        mem_thresh = 0.8*psutil.virtual_memory().available/2**20 # 80% of available memory in MB
        if warn_mem_use and mem_sz > mem_thresh and input(f"{mem_sz:.02f} MB of RAM will be allocated. You have {mem_thresh/.8:.2f} MB available. Continue? [y/n]") != "y":
            raise RuntimeError(f"""Datacube memory allocation ({mem_sz:.02f} MB) exceeded >80% available RAM ({mem_thresh/.8:.2f} MB).
            Halted by user (did not receive `y` at prompt).
            To proceed, you can let `warn_mem_use=False`, decrease `n_lines`, use a `processing_lvl`>=2
            that includes binning, or continue anyway by entering `y` at the prompt.""")
        if self.dc_shape[0] > 1: print(f"Allocated {mem_sz:.02f} MB of RAM. There was {mem_thresh/.8:.2f} MB available.")
        self.dc = CircArrayBuffer(size=self.dc_shape, axis=1, dtype=self.dtype_out)

    def __repr__(self):
        return f"DataCube: shape = {self.dc_shape}, Processing level = {self.proc_lvl}\n"

@patch
def put(self:DataCube, x:np.ndarray):
    """Applies the composed tranforms and writes the 2D array into the data cube. Stores a timestamp for each push."""
    self.timestamps.update()
    self.dc.put( self.pipeline(x) )

@patch
def save(self:DataCube,
         save_dir:str,                 # Path to folder where all datacubes will be saved at
         preconfig_meta_path:str=None, # Path to a .json file that includes metadata fields to be saved inside datacube
         prefix:str="",                # Prepend a custom prefix to your file name
         suffix:str="",                # Append a custom suffix to your file name
        ):
    """Saves to a NetCDF file (and RGB representation) to directory dir_path in folder given by date with file name given by UTC time."""
    if preconfig_meta_path is not None:
        with open(preconfig_meta_path) as json_file:
            attrs = json.load(json_file)
    else: attrs = {}
    if hasattr(self, "ds_metadata"): attrs = self.ds_metadata

    self.directory = Path(f"{save_dir}/{self.timestamps[0].strftime('%Y_%m_%d')}/").mkdir(parents=True, exist_ok=True)
    self.directory = f"{save_dir}/{self.timestamps[0].strftime('%Y_%m_%d')}"

    wavelengths = self.binned_wavelengths if hasattr(self, "binned_wavelengths") else np.arange(self.dc.data.shape[2])

    if hasattr(self,"cam_temperatures"):
        self.coords = dict(wavelength=(["wavelength"],wavelengths),
                           x=(["x"],np.arange(self.dc.data.shape[0])),
                           y=(["y"],np.arange(self.dc.data.shape[1])),
                           time=(["time"],self.timestamps.data.astype(np.datetime64)),
                           temperature=(["temperature"],self.cam_temperatures.data))
    else:
        self.coords = dict(wavelength=(["wavelength"],wavelengths),
                           x=(["x"],np.arange(self.dc.data.shape[0])),
                           y=(["y"],np.arange(self.dc.data.shape[1])),
                           time=(["time"],self.timestamps.data.astype(np.datetime64)))

    # time coordinates can only be saved in np.datetime64 format
    self.nc = xr.Dataset(data_vars=dict(datacube=(["wavelength","x","y"],np.moveaxis(self.dc.data, -1, 0) )),
                         coords=self.coords, attrs=attrs)

    """provide metadata to NetCDF coordinates"""
    self.nc.x.attrs["long_name"]   = "cross-track"
    self.nc.x.attrs["units"]       = "pixels"
    self.nc.x.attrs["description"] = "cross-track spatial coordinates"
    self.nc.y.attrs["long_name"]   = "along-track"
    self.nc.y.attrs["units"]       = "pixels"
    self.nc.y.attrs["description"] = "along-track spatial coordinates"
    self.nc.time.attrs["long_name"]   = "along-track"
    self.nc.time.attrs["description"] = "along-track spatial coordinates"
    self.nc.wavelength.attrs["long_name"]   = "wavelength_nm"
    self.nc.wavelength.attrs["units"]       = "nanometers"
    self.nc.wavelength.attrs["description"] = "wavelength in nanometers."
    if hasattr(self,"cam_temperatures"):
        self.nc.temperature.attrs["long_name"] = "camera temperature"
        self.nc.temperature.attrs["units"] = "degrees Celsius"
        self.nc.temperature.attrs["description"] = "temperature of sensor at time of image capture"

    self.nc.datacube.attrs["long_name"]   = "hyperspectral datacube"
    self.nc.datacube.attrs["units"]       = "digital number"
    if self.proc_lvl in (4,5,7): self.nc.datacube.attrs["units"] = "uW/cm^2/sr/nm"
    elif self.proc_lvl in (6,8): self.nc.datacube.attrs["units"] = "percentage reflectance"
    self.nc.datacube.attrs["description"] = "hyperspectral datacube"

    #self.nc.to_netcdf(f"{self.directory}/{prefix}{self.timestamps[0].strftime('%Y_%m_%d-%H_%M_%S')}{suffix}.nc")
    self.nc.to_netcdf(f"{self.directory}/scan2.nc")

    fig = self.show("matplotlib",hist_eq=True,quick_imshow=True)
    fig.savefig(f"{self.directory}/{prefix}{self.timestamps[0].strftime('%Y_%m_%d-%H_%M_%S')}{suffix}.png",
               bbox_inches='tight', pad_inches=0)

@patch
def load_nc(self:DataCube,
            nc_path:str,            # Path to a NetCDF4 file
            old_style:bool = False, # Only for backwards compatibility for datacubes created before first release
            warn_mem_use:bool = True, # Raise error if trying to allocate too much memory (> 80% of available RAM)
           ):
    """Lazy load a NetCDF datacube into the DataCube buffer."""
    with xr.open_dataset(nc_path) as ds:

        mem_sz = 4*reduce(lambda x,y: x*y, ds.datacube.shape)/2**20 # MB
        mem_thresh = 0.8*psutil.virtual_memory().available/2**20 # 80% of available memory in MB
        if warn_mem_use and mem_sz > mem_thresh and input(f"{mem_sz:.02f} MB of RAM will be allocated. You have {mem_thresh/.8:.2f} MB available. Continue? [y/n]") != "y":
            raise RuntimeError(f"""Datacube load buffer memory allocation ({mem_sz:.02f} MB) exceeded >80% available RAM ({mem_thresh/.8:.2f} MB).
            Halted by user (did not receive `y` at prompt). To proceed, you can let `warn_mem_use=False`, or continue anyway by entering `y` at the prompt.""")

        if old_style: # cross-track, along-track, wavelength
            self.dc      = CircArrayBuffer(size=ds.datacube.shape, axis=1, dtype=type(np.array(ds.datacube[0,0])[0]))
            self.dc.data = np.array(ds.datacube)
        else: # wavelength, cross-track, along-track -> convert to old_style (datacube inserts do not need transpose)
            shape = (*ds.datacube.shape[1:],ds.datacube.shape[0])
            self.dc      = CircArrayBuffer(size=shape, axis=1, dtype=type(np.array(ds.datacube[0,0])[0]))
            self.dc.data = np.moveaxis(np.array(ds.datacube), 0, -1)
        print(f"Allocated {mem_sz:.02f} MB of RAM for the load buffer. There was {mem_thresh/.8:.2f} MB available.")

        self.ds_timestamps = ds.time.to_numpy() # type is np.datetime64. convert to datetime.datetime
        unix_epoch = np.datetime64(0, 's')
        one_second = np.timedelta64(1, 's')
        seconds_since_epoch = (self.ds_timestamps - unix_epoch) / one_second
        self.ds_timestamps = np.array([datetime.utcfromtimestamp(s) for s in seconds_since_epoch])
        self.timestamps.data = self.ds_timestamps
        self.ds_metadata = ds.attrs

        if hasattr(ds,"temperature"):
            self.ds_temperatures = ds.temperature.to_numpy()
            self.cam_temperatures = CircArrayBuffer(size=self.ds_temperatures.shape,dtype=np.float32)
            self.cam_temperatures.data = self.ds_temperatures
        self.binned_wavelengths = np.array(ds.wavelength)
        self.dc.slots_left      = 0 # indicate that the data buffer is full

@patch
def show(self:DataCube,
         plot_lib:str = "bokeh", # Plotting backend. This can be 'bokeh' or 'matplotlib'
         red_nm:float = 640.,    # Wavelength in nm to use as the red
         green_nm:float = 550.,  # Wavelength in nm to use as the green
         blue_nm:float = 470.,   # Wavelength in nm to use as the blue
         robust:bool = False,    # Choose to plot using the 2-98% percentile. Robust to outliers
         hist_eq:bool = False,   # Choose to plot using histogram equilisation
         quick_imshow:bool = False, # Used to skip holoviews and use matplotlib for a static plot
         **plot_kwargs,          # Any other plotting options to be used in your plotting backend
        ) -> "bokeh or matplotlib plot":
    """Generate an RGB image from chosen RGB wavelengths with histogram equalisation or percentile options.
    The plotting backend can be specified by `plot_lib` and can be "bokeh" or "matplotlib".
    Further customise your plot with `**plot_kwargs`. `quick_imshow` is used for saving figures quickly
    but cannot be used to make interactive plots. """

    rgb = np.zeros( (*self.dc.data.shape[:2],3), dtype=np.float32)
    if hasattr(self, "binned_wavelengths"):
        rgb[...,0] = self.dc.data[:,:,np.argmin(np.abs(self.binned_wavelengths-red_nm))]
        rgb[...,1] = self.dc.data[:,:,np.argmin(np.abs(self.binned_wavelengths-green_nm))]
        rgb[...,2] = self.dc.data[:,:,np.argmin(np.abs(self.binned_wavelengths-blue_nm))]
    else:
        rgb[...,0] = self.dc.data[:,:,int(self.dc.data.shape[2] / 2)]
        rgb[...,1] = self.dc.data[:,:,int(self.dc.data.shape[2] / 2)]
        rgb[...,2] = self.dc.data[:,:,int(self.dc.data.shape[2] / 2)]

    if robust and not hist_eq: # scale everything to the 2% and 98% percentile
        vmax = np.nanpercentile(rgb, 98)
        vmin = np.nanpercentile(rgb, 2)
        rgb = ((rgb.astype("f8") - vmin) / (vmax - vmin)).astype("f4")
        rgb = np.minimum(np.maximum(rgb, 0), 1)
    elif hist_eq and not robust:
        img_hist, bins = np.histogram(rgb.flatten(), 256, density=True)
        cdf = img_hist.cumsum() # cumulative distribution function
        cdf = 1. * cdf / cdf[-1] # normalize
        img_eq = np.interp(rgb.flatten(), bins[:-1], cdf) # find new pixel values from linear interpolation of cdf
        rgb = img_eq.reshape(rgb.shape)
    elif robust and hist_eq:
        warnings.warn("Cannot mix robust with histogram equalisation. No RGB adjustments will be made.",stacklevel=2)
        rgb /= np.max(rgb)
    else:
        rgb /= np.max(rgb)

    if quick_imshow:
        fig, ax = plt.subplots(figsize=(12,3))
        ax.imshow(rgb,aspect="equal"); ax.set_xlabel("along-track"); ax.set_ylabel("cross-track")
        return fig

    import holoviews as hv
    hv.extension(plot_lib,logo=False)
    rgb_hv = hv.RGB((np.arange(rgb.shape[1]),np.arange(rgb.shape[0]),
                     rgb[:,:,0],rgb[:,:,1],rgb[:,:,2]))

    if plot_lib == "bokeh":
        return rgb_hv.opts(width=1000,height=250,frame_height=int(rgb.shape[0]//3)).opts(**plot_kwargs).opts(
            xlabel="along-track",ylabel="cross-track",invert_yaxis=True)
    else: # plot_lib == "matplotlib"
        return rgb_hv.opts(fig_inches=22).opts(**plot_kwargs).opts(
            xlabel="along-track",ylabel="cross-track",invert_yaxis=True)
