#!/usr/bin/env python3
import os
import rospy
from mil_openhsi.srv import specSig
from openhsi_deps.cameras import *
from openhsi_deps.data import *
from openhsi_deps.atmos import *
from openhsi_deps.capture import *
from openhsi_deps.calibrate import *
import numpy as np

PATH = os.getenv('MIL_WS')
print('1')
json_path = PATH + "/src/mil/InvestiGator/perception/mil_openhsi/OpenHSI-18_settings_Mono12_bin2.json"
pkl_path  = PATH + "/src/mil/InvestiGator/perception/mil_openhsi/OpenHSI-18_calibration_Mono12_bin2.pkl"

print('2')

lum_preset_dict = {
    0: 1,
    1000: 2,
    2000: 3,
    3000: 4,
    4000: 5,
    5000: 6,
    6000: 7,
    7000: 8,
    8000: 9,
    9000: 10,
    10000: 11,
    20000: 12,
    25000: 13,
    30000: 15,
    35000: 15,
    40000: 16,
}


class OpenHSIServer:
        def __init__(self):
                self.reset_service = rospy.Service("/take_image", specSig, self.image_callback)

        def image_callback(self, req):
                print('3')
                class CalibrateCamera(SettingsBuilderMixin, LucidCamera):
                    pass

                luminances = np.fromiter(lum_preset_dict.keys(), dtype=int)

                exposures = [0,5,8,10,15,20]

                spt = SpectraPTController()

                print('ok')
                with CalibrateCamera(json_path=json_path, pkl_path=pkl_path,processing_lvl=4
                ) as cam:
                        cam.calibration["rad_ref"] = cam.update_intsphere_cube(
                        exposures,luminances,nframes=50)
                        print('urmom')
                        #cam.calibration["rad_ref"] = cam.calibration["rad_ref"].where(
                        #    ~(
                        #        np.sum((cam.calibration["rad_ref"][:, :, :, :, :] == 255), axis=(1, 2))
                        #        > 1000
                        #     )
                        #)
                        cam.dump(json_path=json_path_target, pkl_path=pkl_path_target)
                        cam.collect()
                        print(type(cam.dc))
                        for i in range(cam.dc.size[2]):
                                print(cam.dc[220][500][i])
                        cam.save("hs_scan")
                        #dc2process  = ProcessDatacube(fname="scan.nc", processing_lvl= 4, 
                        #                               json_path= json_path, pkl_path = pkl_path)
                        #dc2process.load_next_tfms([proced_dc.dn2rad])
                        #dc2process.collect()
                        #dc2process.save("dn2rad")
                print('4')
                return ([1,1,1,1,1],)

        # Request is nothing (above line), Repsonse is below line (what we fill out)
        #---
        #bool success
        #string message

        #if req.data:
        #    self.counter = 0
        #    return True, "Counter has been successfully reset"
        #return False, "Counter has not been reset"

if __name__ == '__main__':
    rospy.init_node('openhsi_server')
    OpenHSIServer()
    rospy.spin()

