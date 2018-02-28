import rospy
import cv2
import bdd.msg as BDDMsg
from sensor_msgs.msg import Image
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
    def __init__(self, model_info, node_num):
        self.info = model_info
        self.node_num = node_num

    def startProcess(self):
####        self.model = self.info['model_code']
        self.bridge = CvBridge()
        # create the pytorch model
        rospy.Subscriber('bdd/dual_image', BDDMsg.BDDDualImage, self.output_controls)
        self.controls_pub = rospy.Publisher('bdd/controls/node{0}'.format(self.node_num), BDDMsg.BDDControlsMsg, queue_size=0)

    def output_controls(self, dual_image):
        try:
            left = self.bridge.imgmsg_to_cv2(dual_image.left)
            right = self.bridge.imgmsg_to_cv2(dual_image.right)
            speed = 0.1
            direction = 0.2
            self.controls_pub.publish(BDDMsg.BDDControlsMsg(speed=speed, direction=direction))
        except CvBridgeError as e:
            rospy.logdebug(e)
