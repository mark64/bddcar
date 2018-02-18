import pyzed.camera as zcam
import pyzed.core as mat
import pyzed.defines as sl
import pyzed.types as types
import pyzed.mesh as mesh
import numpy as np

class Camera():
    def __init__(self, mem_type=mat.MEM.CPU, test_mode=False):
        self.initialized = False
        self.test_mode = test_mode
        self.mem_type = mem_type
        self.fps = fps

        init = zcam.PyInitParameters()
        init.camera_resolution = sl.PyRESOLUTION.PyRESOLUTION_HD720
        init.camera_fps = self.fps
        init.depth_mode = sl.PyDEPTH_MODE.PyDEPTH_MODE_NONE
        init.sdk_verbose = True

        self.zed_left = mat.PyMat()
        self.zed_right = mat.PyMat()
        self.cam = zcam.PyZEDCamera()
        status = self.cam.open(init)
        if status != types.PyERROR_CODE.PySUCCESS:
            print("failed to open camera..is the Zed camera connected?")
            if not self.test_mode:
                exit(1)
        else:
            self.initialized = True
            self.runtime_params = zcam.PyRuntimeParameters()

    def __del__(self):
        if self.initialized:
            self.cam.close()

    def capture(self):
        if self.initialized and zed.grab(self.runtime_params) == types.PyERROR_CODE.PySUCCESS:
            zed.retrieve_image(self.zed_left, sl.PyVIEW.PyVIEW_LEFT, self.mem_type)
            zed.retrieve_image(self.zed_right,  sl.PyVIEW.PyVIEW_RIGHT, self.mem_type)
            return True
        if self.test_mode:
            self.left = np.zeros((1080, 720))
            self.right = np.zeros((1080, 720))
        return self.test_mode
