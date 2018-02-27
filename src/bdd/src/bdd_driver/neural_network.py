import rospy
import cv2
from bdd.msg import BDDMsg
from sensor_msg.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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

class Controls():
    def __init__(self, speed, direction):
        self.speed = speed
        self.direction = direction

class NeuralNetwork():
    def __init__(self, model_info):
        self.info = model_info

    def startProcess():
        self.model = self.info['model_code']
        self.bridge = CvBridge()
        # create the pytorch model
        rospy.Subscriber('bdd/dual_image', BDDMsg.DualImage, self.output_controls)

    def output_controls(self, dual_image):
        try:
            left = self.bridge.imgmsg_to_cv2(dual_image.left)
            right = self.bridge.imgmsg_to_cv2(dual_image.right)
        except CvBridgeError as e:
            rospy.logdebug(e)
        speed = 0.1
        direction = 0.2
        return Controls(speed, direction)
