import os
import threading
import time
import rospy
import autocomm.msg as amsg
import autocomm_car.arduino_utils as utils
from autocomm_params import params

class Car():
    def __init__():
        if not self.init_hardware():
            print('failed to initialized car hardware')
            exit(1)
        self.speed = 0 # -1 = max speed back, 0 = stop, 1 = max speed forward
        self.direction = 0 # -1 = left, 0 = straight, 1 = right

    def __del__():
        self.shutdown_hardware()

    def init_hardware():
        utils.init()

    def shutdown_hardware():
        utils.shutdown()

    def update_output():
        utils.write(speed=self.speed, direction=self.direction)

car = Car()

def controls_callback(controls_msg):
    car.speed = controls_msg.speed
    car.direction = controls_msg.direction
    car.update_output()

def main():
    print('starting autocomm car node')
    rospy.init_node('autocomm_car')
    rospy.Subscriber('autocomm_controls', amsg.AutocommControlsMsg, callback=controls_callback)
    rospy.spin()
    print('exiting autocomm car node')

if __name__ == '__main__':
    main()
