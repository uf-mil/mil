#!/usr/bin/env python3
import os
import rospy
from mil_openhsi.srv import specSig
from openhsi_deps.cameras import *
from openhsi_deps.data import *
from openhsi_deps.atmos import *
from openhsi_deps.capture import *
from openhsi_deps.calibrate import *
PATH = os.getenv('MIL_WS')
print('1')
json_path = PATH + "/src/mil/InvestiGator/perception/mil_openhsi/OpenHSI-18_settings_Mono12_bin2.json"
pkl_path  = PATH + "/src/mil/InvestiGator/perception/mil_openhsi/OpenHSI-18_calibration_Mono12_bin2.pkl"

print('2')

class OpenHSIServer:
	def __init__(self):
        	self.reset_service = rospy.Service("/take_image", specSig, self.image_callback)

	def image_callback(self, req):
		
		with LucidCamera(n_lines        = 1_000,
                 processing_lvl = 2,
                 pkl_path       = pkl_path,
                 json_path      = json_path,
                 exposure_ms    = 10
                ) as cam:
			cam.collect()
			print(type(cam.dc))
			for i in range(cam.dc.size[2]):
				print(cam.dc[220][500][i])
			cam.save("hs_scan")
		print("1")
		dc2process  = ProcessDatacube(fname="hs_scan/2022_09_09/scan2.nc",processing_lvl=4,json_path= json_path,pkl_path=pkl_path)
		print("2")
		print(type(dc2process))
		dc2process.load_next_tfms([dc2process.dn2rad])
		print("3")
		dc2process.collect()
		print("4")
		dc2process.save("dn2rad")
		print('4')
		print(dc2process.dc)
		
		radiances = []
		 
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
