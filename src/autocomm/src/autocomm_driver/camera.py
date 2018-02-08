import pyzed.camera as zcam
import pyzed.core as mat
import pyzed.defines as sl
import pyzed.types as types
import pyzed.mesh as mesh
import numpy as np

class Camera():
    def __init__(self, test_mode=False):
        self.initialized = False
        self.test_mode = test_mode
        init = zcam.PyInitParameters()
        init.camera_resolution = sl.PyRESOLUTION.PyRESOLUTION_HD720
        init.camera_fps = 30
        init.depth_mode = sl.PyDEPTH_MODE.PyDEPTH_MODE_NONE
        init.sdk_verbose = True
        self.zed_left = mat.PyMat()
        self.zed_right = mat.PyMat()
        self.left = None
        self.right = None
        self.cam = zcam.PyZEDCamera()
        status = self.cam.open(init)
        if status != types.PyERROR_CODE.PySUCCESS:
            print("failed to open camera..is the Zed camera connected?")
            if not self.test_mode:
                exit(1)
        else:
           self.initialized = True

    def __del__(self):
        if self.initialized:
            self.cam.close()

    def capture(self):
        self.left = None
        self.right = None
        if self.initialized and zed.grab(zcam.PyRuntimeParameters()) == types.PyERROR_CODE.PySUCCESS:
            zed.retrieve_image(self.zed_left, sl.PyVIEW.PyVIEW_LEFT)
            zed.retrieve_image(self.zed_right,  sl.PyVIEW.PyVIEW_RIGHT)
            self.left = self.zed_left.get_data()
            self.right = self.zed_right.get_data()
            return True
        if self.test_mode:
            self.left = np.zeros((1080, 720))
            self.right = np.zeros((1080, 720))
        return self.test_mode
