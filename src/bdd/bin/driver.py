#!/usr/bin/env python
import rospy
from bdd_driver import camera, neural_network
Camera = camera.Camera
NeuralNetwork = neural_network.NeuralNetwork
from params import params

"""
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

def main():
    rospy.init_node('bdd_driver')
    rospy.logdebug('starting bdd driver node')

    cam = Camera(camera_number=0, mem_type=camera.MemType.CPU)
    if not cam.initialized:
        rospy.logfatal('failed to connect to zed camera..exiting')
        exit(1)



    rate = rospy.Rate(cam.fps)
    while not rospy.is_shutdown():
        success, left, right = cam.capture(display=True)
        if cam.capture():
            print('yay')
        rate.sleep()
    rospy.logdebug('bdd driver node exiting')

if __name__ == '__main__':
    main()
