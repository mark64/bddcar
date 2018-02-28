#!/usr/bin/env python
import rospy
from bdd_driver import camera, neural_network, controls_aggregator
Camera = camera.Camera
NeuralNetwork = neural_network.NeuralNetwork
Aggregator = controls_aggregator.Aggregator
from params import params
import bdd.msg as BDDMsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

"""Main node
This is the driver ros node,
the main entry-point for car
control generation. For information
on actually sending controls to
the hardware and other hw/sw
interface code, see car.py and
the bdd_car module

The driver node is responsible for spawning
neural network processes for each
set of parameters as returned by
params.models (see the params module)

In addition to starting the
neural networks, this initializes the
zed camera and takes care of publishing
images to a ros topic, to which the
neural network processes should subscribe
"""

cv_bridge = CvBridge()

def main():
    rospy.init_node('bdd_driver')
    rospy.logdebug('starting bdd driver node')

    cam = Camera(camera_number=0, mem_type=camera.MemType.CPU)
    if not cam.initialized:
        rospy.logfatal('failed to connect to zed camera..exiting')
        exit(1)
    rospy.logdebug('camera initialized')

    net = neural_network.NeuralNetwork({}, 0)
    agg = Aggregator(1)

    image_pub = rospy.Publisher('bdd/dual_image', BDDMsg.BDDDualImage, queue_size=0)
    rate = rospy.Rate(cam.fps)
    net.startProcess()
    while not rospy.is_shutdown():
        success, left, right = cam.capture(display=False)
        if success:
            try:
                ros_left = cv_bridge.cv2_to_imgmsg(left, "bgr8")
                ros_right = cv_bridge.cv2_to_imgmsg(right, "bgr8")
                image_pub.publish(BDDMsg.BDDDualImage(left=ros_left, right=ros_right))
            except CvBridgeError as e:
                rospy.logdebug(e)
        rate.sleep()
    rospy.logdebug('bdd driver node exiting')

if __name__ == '__main__':
    main()
