# AUTOGENERATED! DO NOT EDIT! File to edit: nbs/03_atmos.ipynb (unless otherwise specified).

__all__ = ['Model6SV', 'remap', 'SpectralLibrary', 'SpectralMatcher', 'ELC', 'DataCubeViewer']

# Cell

from fastcore.foundation import patch
from fastcore.meta import delegates
from fastcore.basics import num_cpus
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
import os
import copy
from tqdm import tqdm
import warnings
from pathlib import Path
import pickle

from typing import Iterable, Union, Callable, List, TypeVar, Generic, Tuple, Optional

import param
import panel as pn
import panel.widgets as pnw
pn.extension()

import holoviews as hv
hv.extension('bokeh',logo=False)

from Py6S import *


# Cell

from .data import *

# Cell

class Model6SV(object):
    """Create a 6SV model using Py6S."""
    def __init__(self,
                 lat:float = -17.7, # latitude in degrees
                 lon:float = 146.1, # longitude in degrees
                 z_time:datetime = datetime.strptime("2021-05-26 04:00","%Y-%m-%d %H:%M"), # Zulu datetime
                 station_num:int = 94299, # radiosonde station number
                 region:str = "pac", # radiosonde region code
                 alt:float = 0.12,   # altitude in km
                 zen:float = 0.,     # viewing zenith angle in degrees
                 azi:float = 0.,     # viewing azimuth angle in degrees
                 tile_type:GroundReflectance = 1.0, # ground reflectance for spectralon panel
                 aero_profile:AeroProfile = AeroProfile.Maritime, # 6SV aerosol profile
                 wavelength_array:np.array = None, # wavelengths array in nm
                 sixs_path:str=None, # path to 6SV executable
                ):
        """Calculates the at sensor radiance using 6SV for location at latitude `lat` and longitude `lon` at time `z_time` and altitude `alt`.
        The `station_num` and `region` refers to the radiosonde data. You can also specify the viewing zenith `zen` and azimuth `azi`.
        The radiance is calculated for wavelengths in `wavelength_array`. The 6SV executable path can be specified in `sixs_path`."""

        if wavelength_array is None: wavelength_array = np.arange(400,800,4)
        self.wavelength_array = wavelength_array/1e3 # convert to μm for Py6S
        s = SixS(sixs_path)

        # Atmosphere
        s.atmos_profile = AtmosProfile.PredefinedType(AtmosProfile.MidlatitudeSummer)

        # crude calculation if daytime is 12Z or 0Z based on time and longitude
        z_hour = 0 if ((z_time.hour + int(lon/30))%24 - 12)/12. < 0.5 else 12
        radiosonde_url = f"http://weather.uwyo.edu/cgi-bin/sounding?region={region}&TYPE=TEXT%3ALIST&YEAR={z_time.year}&MONTH={z_time.month:02d}&FROM={z_time.day:02d}{z_hour:02d}{z_time.minute:02d}&TO={z_time.day:02d}{z_hour:02d}&STNM={station_num}"
        s.atmos_profile = SixSHelpers.Radiosonde.import_uow_radiosonde_data(radiosonde_url,AtmosProfile.MidlatitudeSummer)

        # Aerosol
        s.aero_profile = AeroProfile.PredefinedType(aero_profile)
        s.visibility = 40 # km

        #Viewing and sun geometry
        s.geometry = Geometry.User()
        s.geometry.day = z_time.day
        s.geometry.month = z_time.month
        dt_str = f"{z_time.year}-{z_time.month:02d}-{z_time.day:02d} {z_time.hour:02d}:{z_time.minute:02d}:{z_time.second:02d}"
        s.geometry.from_time_and_location(lat, lon, dt_str, zen, azi)

        #Altitude
        s.altitudes = Altitudes()
        s.altitudes.set_sensor_custom_altitude(alt) # km
        s.altitudes.set_target_sea_level()
        if tile_type is not None: s.ground_reflectance = GroundReflectance.HomogeneousLambertian(tile_type)

        self.s = s
        self.__call__()

    def rad2photons(self):
        self.photons = self.radiance/( 1.98644582e-25/(self.wavelength_array*1e-6) )

    def __call__(self):
        """calculate the at sensor radiance"""
        self.radiance = self.run_wavelengths(self.wavelength_array) # units of (W/m^2/sr/μm)

        df = pd.DataFrame({"wavelength":self.wavelength_array,"radiance":self.radiance})
        df.set_index("wavelength",inplace=True)
        df.interpolate(method="cubicspline",axis="index",limit_direction="both",inplace=True)
        self.radiance = df["radiance"].to_numpy()
        self.rad2photons()

    def show(self, plot_lib:str="bokeh" ) -> "figure object":
        """plot the calculated radiance"""
        hv.extension(plot_lib,logo=False)
        return hv.Curve( zip(self.wavelength_array*1e3,self.radiance/10), label="computed radiance").opts(
            xlabel="wavelength (nm)", ylabel="radiance (μW/cm$^2$/sr/nm)")


# Cell

@patch
def _sixs_run_one_wavelength(self:Model6SV, wv:float) -> float:
    """Runs one instance of 6SV for one wavelength wv"""
    self.s.outputs = None
    a = copy.deepcopy(self.s)
    a.wavelength = Wavelength(wv)
    a.run()
    return SixSHelpers.Wavelengths.recursive_getattr(a.outputs, "pixel_radiance")

@patch
def run_wavelengths(self:Model6SV, wavelengths:np.array, n_threads:int = None) -> np.array:
    """Modified version of SixSHelpers.Wavelengths.run_wavelengths that has a progress bar.
    This implementation uses threading (through Python's multiprocessing API)."""
    from multiprocessing.dummy import Pool

    if n_threads is None: n_threads = num_cpus()
    with Pool(n_threads) as p, tqdm(total=len(wavelengths)) as pbar:
        res = [p.apply_async( self._sixs_run_one_wavelength, args=(wavelengths[i],),
                callback=lambda _: pbar.update(1)) for i in range(len(wavelengths))]
        results = [r.get() for r in res]

    return np.array(results)




# Cell
def remap(x, in_min, in_max, out_min, out_max):
    """convert `x` from between input range (`in_min`,`in_max`) to output range (`out_min`,`out_max`)."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Cell

class SpectralLibrary():
    """Manages all the spectral library and interpolations as necessary."""
    def __init__(self,speclib_path:str=None):
        """Loads the spectral library file (which can be .gzip or .pkl) located at `speclib_path`."""
        self.speclib_path = speclib_path
        if speclib_path is not None:
            if "gzip" in speclib_path: self.speclib = pd.read_parquet(speclib_path)
            else: self.speclib = pd.read_pickle(speclib_path)
            self.wavelengths = self.speclib["wavelength"].to_numpy()
        else:
            self.wavelengths = np.arange(350,2501)
            self.speclib = pd.DataFrame(self.wavelengths.reshape(2151,1),columns=["wavelength"])
            warnings.warn("Spectral Library file required! Please re-initialised with the file.",stacklevel=2)

        self.orig_speclib = self.speclib.copy() # Keep a copy of the original data

    def show(self,plot_lib:str="bokeh", topk:pd.DataFrame=None, radiance_6SV:np.array=None) -> "figure object":
        """Plot the spectral library using backend `plot_lib`. If `topk` pd.DataFrame is provided, then only those are plotted.
        If `radiance_6SV` array provided, then plot in radiance instead of reflectance."""
        hv.extension(plot_lib,logo=False)
        if plot_lib == "bokeh": plot_opts = {"width":1000,"height":400,"legend_position":"right","legend_offset":(0,20)}
        elif plot_lib == "matplotlib": plot_opts = {"fig_inches":(12,5)}

        # plot all the reflectance spectra in the library
        if topk is None:
            curve_list = [hv.Curve( zip(self.wavelengths,self.speclib[i]),label=i) for i in self.speclib.columns ]
            if hasattr(self,"last_spectra"):
                return (hv.Overlay(curve_list)*hv.Curve(zip(self.wavelengths,self.last_spectra),label="last spectra")).opts(
                                **plot_opts,xlabel="wavelength (nm)",ylabel="reflectance",ylim=(0,1.1))
            return (hv.Overlay(curve_list)).opts(
                                **plot_opts,xlabel="wavelength (nm)",ylabel="reflectance",ylim=(0,1.1))

        # plot the top matches in reflectance or radiance
        temp = self.speclib[topk["label"]].to_numpy()
        if radiance_6SV is not None:
            temp = (temp.T*radiance_6SV/10).T
        temp = pd.DataFrame(temp,columns=topk["label"])
        temp.insert(0,"wavelength",self.wavelengths)

        curve_list = []
        if hasattr(self,"last_spectra") and hasattr(self,"rad_6SV"):
            if radiance_6SV is not None:
                curve_list.append( hv.Curve(zip(self.wavelengths,self.last_spectra),label="tap point") )
            else: curve_list.append( hv.Curve(zip(self.wavelengths,self.last_spectra/self.rad_6SV),label="6SV estimate") )
        alphas = topk["score"].to_numpy()
        alphas = np.exp(alphas)/np.sum(np.exp(alphas)) # softmax the transparency alpha to bring out the closest match
        alphas = remap(alphas,np.min(alphas),np.max(alphas),0.1,0.9999)

        for i in range(len(topk["label"])):
            curve_list.append( hv.Curve(temp,kdims="wavelength",vdims=topk["label"][i],label=topk["label"][i]).opts(alpha=alphas[i]) )
        if radiance_6SV is not None:
            return hv.Overlay(curve_list).opts(**plot_opts,xlabel="wavelength (nm)",
                                               ylabel="radiance (uW/cm^2/sr/nm)",ylim=(0,1.1*np.max(radiance_6SV)/10),
                                               title=f"top match: {topk['label'][0]}")
        return hv.Overlay(curve_list).opts(**plot_opts,xlabel="wavelength (nm)",ylabel="reflectance",ylim=(0,1.1),
                                           title=f"top match: {topk['label'][0]}")

    def interp(self, wavelengths:np.array):
        """Interpolate all the spectra in the library to the new `wavelengths`."""
        self.wavelengths = wavelengths
        self.speclib = self.orig_speclib.copy()
        self.speclib.insert(0,"type","USGS")
        self.speclib = pd.concat( [ pd.DataFrame({"type":"openhsi","wavelength":wavelengths}), self.speclib] )
        self.speclib.set_index("wavelength",inplace=True)
        self.speclib.interpolate(method="cubicspline",axis="index",limit_direction="both",inplace=True)
        self.speclib = self.speclib[self.speclib["type"].str.match("openhsi")]
        self.speclib.drop(labels="type", axis=1,inplace=True)

    def dump(self, save_path:str=None):
        """Dump the spectral library to file. If no `save_path` provided, overwrite the original file."""
        if save_path is not None:
            if "gzip" in save_path: self.orig_speclib.to_parquet(save_path,compression="gzip")
            else: self.orig_speclib.to_pickle(save_path,protocol=4)
            print(f"Updated your spectral library at {save_path}")
        else:
            if "gzip" in self.speclib_path: self.orig_speclib.to_parquet(self.speclib_path,compression="gzip")
            else: self.orig_speclib.to_pickle(self.speclib_path,protocol=4)
            print(f"Overwritten your spectral library at {self.speclib_path}")



# Cell

@patch
def import_USGS(self:SpectralLibrary,directory:str,sort:bool=False):
    """Import the ASD files from `directory` with optional names `sort`ed."""
    fnames = sorted(os.listdir(directory))
    # only include the ASD instrument for now (other instruments have different wavelength ranges)
    fnames = [f for f in fnames if "ASD" in f]
    col_names = [f[9:-16] for f in fnames]
    cwd = Path(directory)

    data = []
    for f in fnames:
        temp = pd.read_csv(cwd/f,delimiter="\t")
        temp.loc[~(temp[temp.columns[0]] > 0), temp.columns[0]] = np.nan # replace any negative reflectance with NaNs
        data.append(temp[temp.columns[0]].to_numpy())

    df = pd.DataFrame(np.array(data).T,columns=col_names)
    self.orig_speclib = pd.concat([self.orig_speclib,df],axis=1)
    self.orig_speclib = self.orig_speclib.loc[:,~self.orig_speclib.columns.duplicated()]
    if sort:
        self.orig_speclib = self.orig_speclib.reindex(sorted(self.orig_speclib.columns), axis=1,)

    self.interp(self.wavelengths)
    print(f"Added folder of ASD spectra to spectral library")

# Cell

from numpy.linalg import norm

@delegates()
class SpectralMatcher(SpectralLibrary):
    """Match OpenHSI spectra against spectral library using Spectral Angle Mapper algorithm"""
    def __init__(self,pkl_path:str,**kwargs):
        """Prepare arrays for spectral matching in radiance.
        `pkl_path` is needed to get the openhsi wavelengths and precomputed 6SV radiance fit."""
        super().__init__(**kwargs)

        with open(pkl_path,'rb') as handle:
            self.calibration = pickle.load(handle)
        self.interp(self.calibration["wavelengths"])

    def interp(self,wavelengths:np.array):
        """Interpolate to new wavelengths and update some vectors."""
        super().interp(wavelengths)
        self.rad_6SV = np.float32(self.calibration["rad_fit"]( self.wavelengths ))

        self.speclib_ref  = self.speclib.copy()                # reflectances
        self.spectra      = (self.speclib[:].T*self.rad_6SV).T # radiances
        self.spectra_norm = norm(self.spectra,axis=0)          # vector lengths

    def show(self,plot_lib:str="bokeh", is_rad:bool=False) ->"figure object":
        """Plot the spectral library using backend `plot_lib`. Default is to show in reflectance. Set `is_rad` to show in radiance."""
        if is_rad and hasattr(self,"sim_df"):
            return super().show(plot_lib,topk=self.sim_df,radiance_6SV=self.rad_6SV*10)
        elif not is_rad and hasattr(self,"sim_df"):
            return super().show(plot_lib,topk=self.sim_df)
        elif is_rad:
            return super().show(plot_lib,radiance_6SV=self.rad_6SV*10)
        else:
            return super().show(plot_lib)

    def topk_spectra(self,spectrum:np.array,k:int=5,refine=True):
        """Match a `spectrum` against a spectral library `spectra`. Return the top k.
        Set `refine` to find closest match using mean squared error on the top k results."""
        self.refine = refine

        self.last_spectra = np.array(spectrum)
        cosine_dist       = self.last_spectra @ self.spectra / ( norm(self.last_spectra) * self.spectra_norm ) # less than O(n^3)
        topk_idx          = np.argpartition(cosine_dist, -k)[-k:]             # linear time rather than n log n
        self.topk_idx     = topk_idx[np.argsort(cosine_dist[topk_idx])][::-1] # k log k

        if refine and k > 1:
            topk_spectra    = self.spectra[ self.speclib.columns[self.topk_idx] ].to_numpy().transpose()
            residuals       = norm(topk_spectra - self.last_spectra, axis=1)
            residuals       = remap(residuals, min(residuals),max(residuals),0,0.1)
            subset_topk_idx = np.argsort(cosine_dist[self.topk_idx] - residuals)[::-1]
            self.topk_idx   = self.topk_idx[subset_topk_idx]

            self.sim_df = pd.DataFrame({"label":self.speclib.columns[self.topk_idx],"score":cosine_dist[self.topk_idx]-residuals[subset_topk_idx]})
            return self.sim_df

        self.sim_df = pd.DataFrame({"label":self.speclib.columns[self.topk_idx],"score":cosine_dist[self.topk_idx]})
        return self.sim_df


# Cell

@patch
def topk_spectra(self:SpectralMatcher,spectrum:np.array,k:int=5,refine=True):
    """Match a `spectrum` against a spectral library `spectra`. Return the top k.
    Set `refine` to find closest match using mean squared error on the top k results."""
    self.refine = refine

    self.last_spectra = np.array(spectrum)
    cosine_dist       = self.last_spectra @ self.spectra / ( norm(self.last_spectra) * self.spectra_norm ) # less than O(n^3)
    topk_idx          = np.argpartition(cosine_dist, -k)[-k:]             # linear time rather than n log n
    self.topk_idx     = topk_idx[np.argsort(cosine_dist[topk_idx])][::-1] # k log k

    if refine and k > 1:
        topk_spectra    = self.spectra[ self.speclib.columns[self.topk_idx] ].to_numpy().transpose()
        residuals       = norm(topk_spectra - self.last_spectra, axis=1)
        residuals       = remap(residuals, min(residuals),max(residuals),0,0.1)
        subset_topk_idx = np.argsort(cosine_dist[self.topk_idx] - residuals)[::-1]
        self.topk_idx   = self.topk_idx[subset_topk_idx]

        self.sim_df = pd.DataFrame({"label":self.speclib.columns[self.topk_idx],"score":cosine_dist[self.topk_idx]-residuals[subset_topk_idx]})
        return self.sim_df

    self.sim_df = pd.DataFrame({"label":self.speclib.columns[self.topk_idx],"score":cosine_dist[self.topk_idx]})
    return self.sim_df

# Cell

@delegates()
class ELC(SpectralMatcher):
    """Apply ELC for radiance datacubes"""
    def __init__(self,nc_path:str,old_style:bool=False,**kwargs):
        """Load datacube at `nc_path` and setup UI."""

        self.nc_path = nc_path
        self.dc = DataCube(processing_lvl=-1)
        self.dc.load_nc(nc_path,old_style)
        self.RGB = self.dc.show("bokeh",robust=True).opts(height=250, width=1000, invert_yaxis=True,tools=["tap"],toolbar="below")
        self.a_ELC = np.ones((self.dc.dc.data.shape[-1],))
        self.b_ELC = np.zeros((self.dc.dc.data.shape[-1],))
        super().__init__(**kwargs)
        self.interp(self.dc.binned_wavelengths)
        self.xx = np.zeros((len(self.wavelengths),2))
        self.data = self.dc.dc.data.copy() # so saving is done on a different datacube

        self.title_txt = pn.pane.Markdown("**Interactive Empirical Line Calibrator**",)
        self.event_msg = pnw.StaticText(name="", value="Click export buttons to save desired reflectance datacubes.")
        self.export_ELC_button = pnw.Button(name="Export ELC datacube",button_type="primary",width=150,height=25)
        self.export_6SV_button = pnw.Button(name="Export 6SV datacube",button_type="primary",width=150,height=25)


    def __call__(self):
        """Setup button callbacks and interactive stream and dynamics plots."""
        self.setup_export_ELC()
        self.setup_export_6SV()
        self.setup_streams()
        self.setup_callbacks()

        return pn.Column( self.title_txt,
                          self.RGB * self.boxes * self.ELC_dmap * self.hit_dmap * self.ch_dmap,
                          pn.Tabs( ("Reflectance", self.ref_curve), ("Radiance",self.rad_curve),dynamic=True),
                          pn.Row(self.export_ELC_button, self.export_6SV_button, self.event_msg) )

    def setup_export_ELC(self):
        """Setup ELC export button callback"""
        def click_func(event):
            # compute the ELC datacube
            self.event_msg.value = f"saving..."
            self.dc.dc.data[...] = (self.data - self.b_ELC)/self.a_ELC
            self.dc.timestamps.data = self.dc.ds_timestamps
            save_dir = "/".join(self.nc_path.split("/")[:-1])
            self.dc.save(save_dir,suffix="_ELC")
            self.event_msg.value = f"ELC datacube exported to directory {save_dir}. Buttom clicked {self.export_ELC_button.clicks} time(s)"

        self.export_ELC_button.on_click(click_func)


    def setup_export_6SV(self):
        """Setup 6SV export button callback"""
        def click_func(event):
            # compute the 6SV datacube
            self.event_msg.value = f"saving..."
            self.dc.dc.data[...] = self.data/self.rad_6SV
            self.dc.timestamps.data = self.dc.ds_timestamps
            save_dir = "/".join(self.nc_path.split("/")[:-1])
            self.dc.save(save_dir,suffix="_6SV")
            self.event_msg.value = f"6SV datacube exported to directory {save_dir}. Buttom clicked {self.export_6SV_button.clicks} time(s)"

        self.export_6SV_button.on_click(click_func)

    def setup_streams(self):
        """Setup the mouseclick position and bounding box input streams."""
        # Declare Tap stream with heatmap as source and initial values
        self.posxy = hv.streams.Tap(source=self.RGB)

        # Declare pointer stream initializing at (0, 0) and linking to Image
        self.pointer = hv.streams.PointerXY(x=0, y=0, source=self.RGB)

        self.boxes = hv.Rectangles([]).opts(active_tools=['box_edit'], fill_alpha=0.5)
        self.box_stream = hv.streams.BoxEdit(source=self.boxes, num_objects=5, styles={'fill_color': 'red'})

    def setup_callbacks(self):
        """Setup the mouseover crosshair, and mouseclick callbacks."""
        # update 'x' marker on mouseclick
        def hit_mark(x,y):
            return hv.Scatter((x,y)).opts(color='r', marker='x', size=20)
        self.hit_dmap = hv.DynamicMap(hit_mark, streams=[self.posxy])

        # Draw cross-hair at cursor position
        def cross_hair_info(x, y):
            return hv.HLine(y) * hv.VLine(x)
        self.ch_dmap = hv.DynamicMap(cross_hair_info, streams=[self.pointer])

        def tap_radiance(x,y):
            if x is None or y is None:
                x = 0; y = 0
            x = int(x); y = int(y)

            sim_df = self.topk_spectra(np.array(self.data[y,x,:]),5,refine=True)

            return self.show(is_rad=True).opts(
                legend_position='right', legend_offset=(0, 20),title=f'top match: {self.sim_df["label"][0]}, score: {self.sim_df["score"][0]:.3f}, position=({y:d},{x:d})').opts(framewise=True,
                xlabel="wavelength (nm)",ylabel="radiance (uW/cm^2/sr/nm)",height=400, width=1000,ylim=(0,np.max(self.spectra["spectralon"])))
        self.rad_curve =  hv.DynamicMap(tap_radiance, streams=[self.posxy]).opts(shared_axes=False,height=250)

        def tap_reflectance(x, y):
            if x is None or y is None:
                x = 0; y = 0
            x = int(x); y = int(y)

            spectra = (self.data[y,x,:] - self.b_ELC)/self.a_ELC
            sim_df = self.topk_spectra(self.data[y,x,:],5,refine=True)

            return (hv.Curve(zip(self.wavelengths,spectra),label="ELC estimate") * self.show(is_rad=False)).opts(axiswise=True,
                legend_position='right', legend_offset=(0, 20),title=f'top match: {self.sim_df["label"][0]}, score: {self.sim_df["score"][0]:.3f}, position=({y:d},{x:d})').opts(framewise=True,
                ylabel="reflectance",xlabel="wavelength (nm)",yaxis="left",height=400, width=1000,ylim=(0,1.05))

        # Connect the Tap stream to the tap_spectra callback
        self.ref_curve = hv.DynamicMap(tap_reflectance, streams=[self.posxy]).opts(shared_axes=False,height=250)

        def update_ELC(data):
            if data is None or len(data) == 0: return hv.Curve([])

            # build up the matrices
            A = []; b = []
            data = zip(np.int32(data['x0']), np.int32(data['x1']), np.int32(data['y0']), np.int32(data['y1']) )
            for i, (x0, x1, y0, y1) in enumerate(data):
                if y1 > y0: y0, y1 = y1, y0
                sz = ((y0-y1)*(x1-x0),len(self.wavelengths))
                selection = np.reshape(np.array(self.data[y1:y0,x0:x1,:]),sz)

                AA = np.zeros((len(self.wavelengths),sz[0],2))
                bb = np.zeros((len(self.wavelengths),sz[0],1))

                for j in range(sz[0]):
                    self.topk_spectra(selection[j,:],k=5,refine=True)
                    label = self.sim_df["label"][0]
                    AA[:,j,0] = self.speclib[label].to_numpy()
                    bb[:,j,0] = selection[j,:]
                AA[:,:,1] = 1

                A.append(AA); b.append(bb)

            A = np.concatenate(A,axis=1); b = np.concatenate(b,axis=1)
            x = np.linalg.pinv(A) @ b

            self.xx[:,0] = x[:,0,0]; self.xx[:,1] = x[:,1,0]
            self.a_ELC = self.xx[:,0]; self.b_ELC = self.xx[:,1]
            return hv.Curve([])
        self.ELC_dmap = hv.DynamicMap(update_ELC, streams=[self.box_stream])




# Cell

class DataCubeViewer():
    """Explore datacubes
        Optional key/val pair arguement overrides (**kwargs)
        ylim
        ylabel
    """
    def __init__(self,
                 nc_path:str=None, # path to the NetCDF file
                 old_style:bool=False, # if datacube is stored as cross-track, along-track, wavelength coords
                 img_aspect_ratio:float=0.25, # aspect ratio for the datacube viewer
                 box_sz:tuple=(1,1), # Any binning (nrows, ncols) around the tap point. Default is a single pixel
                 **kwargs):
        """Load datacube at `nc_path` and setup UI."""

        self.nc_path = nc_path
        self.dc      = DataCube(processing_lvl=-1)
        self.dc.load_nc(nc_path,old_style)
        self.RGB     = self.dc.show("bokeh",robust=True).opts(height=int(1000*img_aspect_ratio), width=1000, invert_yaxis=True,tools=["tap"],toolbar="below")
        self.ylim    = (0,np.max(self.dc.dc.data))
        self.box_sz  = box_sz
        self._bhalf = (box_sz[0]//2, box_sz[1]//1)

        if 'ylabel' in kwargs:
            self.ylabelplot=kwargs.get("ylabel")
        else:
            if type(self.dc.dc.data.dtype) is np.float32:
                self.ylabelplot = "radiance (uW/cm^2/sr/nm)"
                if self.ylim[1] < 2:
                    self.ylabelplot = "reflectance"
            else: self.ylabelplot = "digital number"

        if 'ylim' in kwargs: self.ylim=kwargs.get("ylim")
        else: self.ylim = (0,np.max(self.dc.dc.data))

        self.title_txt = pn.pane.Markdown("**Interactive Datacube Viewer**",)

    def __call__(self):
        """Setup button callbacks and interactive stream and dynamics plots."""
        self.setup_streams()
        self.setup_callbacks()

        return pn.Column( self.title_txt,
                          self.RGB * self.hit_dmap * self.ch_dmap,
                          self.tap_curve )

    def setup_streams(self):
        """Setup the mouseclick position and bounding box input streams."""
        # Declare Tap stream with heatmap as source and initial values
        self.posxy = hv.streams.Tap(source=self.RGB)

        # Declare pointer stream initializing at (0, 0) and linking to Image
        self.pointer = hv.streams.PointerXY(x=0, y=0, source=self.RGB)

    def setup_callbacks(self):
        """Setup the mouseover crosshair, and mouseclick callbacks."""
        # update 'x' marker on mouseclick
        def hit_mark(x,y):
            return hv.Scatter((x,y)).opts(color='r', marker='x', size=20)
        self.hit_dmap = hv.DynamicMap(hit_mark, streams=[self.posxy])

        # Draw cross-hair at cursor position
        def cross_hair_info(x, y):
            return hv.HLine(y) * hv.VLine(x)
        self.ch_dmap = hv.DynamicMap(cross_hair_info, streams=[self.pointer])

        def tap(x,y):
            if x is None or y is None:
                x = 1; y = 1
            x = int(x); y = int(y)

            x_slice = slice( np.max((x-self._bhalf[1],0)), np.min((x-self._bhalf[1]+self.box_sz[1],self.dc.dc.data.shape[1])) )
            y_slice = slice( np.max((y-self._bhalf[0],0)), np.min((y-self._bhalf[0]+self.box_sz[0],self.dc.dc.data.shape[0])) )
            c = np.mean(self.dc.dc.data[y_slice,x_slice,:],axis=(0,1))
            #c = self.dc.dc.data[y,x,:]

            return hv.Curve( zip(self.dc.binned_wavelengths,c), label=f"tap point at ({x},{y}),").opts(xlabel="wavelength (nm)",ylabel=self.ylabelplot,
                                                                                       ylim=self.ylim)
        self.tap_curve =  hv.DynamicMap(tap, streams=[self.posxy]).opts(shared_axes=False,height=250,width=1000)
