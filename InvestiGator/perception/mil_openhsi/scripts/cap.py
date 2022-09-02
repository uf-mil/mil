from deps.cameras import *
from deps.data import *
json_path = "deps/OpenHSI-18_settings_Mono12_bin2.json"
pkl_path  = "deps/OpenHSI-18_calibration_Mono12_bin2.pkl"

with LucidCamera(n_lines        = 1_000, 
                 processing_lvl = 2, 
                 pkl_path       = pkl_path,
                 json_path      = json_path,
                 exposure_ms    = 10
                ) as cam:
    cam.collect()
    cam.save("hs_scan")
    fig = cam.show(plot_lib="matplotlib", robust=True)
