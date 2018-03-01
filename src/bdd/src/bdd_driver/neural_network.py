import rospy
import cv2
import bdd.msg as BDDMsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import torch

"""Neural Network class
Run a neural network based on the
parameters in the dictionary model_info.

This is framework agnostic; it does not
matter whether you use pytorch,
tensorflow, numpy, caffe, or raw C
to implement your neural network, you
just need to implement a simple python module
that can be called from output_controls and
returns a speed and direction given
image input

see the examples in <TODO: make examples dir>
"""

class NeuralNetwork():
    def __init__(self, model_info, node_num):
        self.info = model_info
        self.node_num = node_num

    def start_process(self):
####        self.model = self.info['model_code']
        self.bridge = CvBridge()
        # create the pytorch model
        rospy.Subscriber('bdd/dual_image', BDDMsg.BDDDualImage, self.image_callback)
        self.controls_pub = rospy.Publisher('bdd/controls/node{0}'.format(self.node_num), BDDMsg.BDDControlsMsg, queue_size=0)

    def image_callback(self, dual_image):
        try:
            image = self.bridge.imgmsg_to_cv2(dual_image.combined_image)
            speed, direction = self.output_contrls(image)
            self.controls_pub.publish(BDDMsg.BDDControlsMsg(speed=speed, direction=direction))
        except CvBridgeError as e:
            rospy.logdebug(e)

    def init_model(self):
        pass

    def output_controls(self, image):
        speed = 0.0
        direction = 0.0
        return speed, direction

    def numpy_int_mat_to_pytorch_float_tensor(np_mat):
        """helper function, incomplete
        """
        tensor = torch.from_numpy(np_mat)
        tensor = (tensor.float() / 255.0) - 0.5
        # TODO: add more processing code here
        return tensor
