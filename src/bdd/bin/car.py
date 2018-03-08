#!/usr/bin/env python
import os
import threading
import time
import serial
import rospy
import bdd.msg as BDDMsg
from params import params

class Car():
    """Car class
    takes care of interfacing with the arduino
    and translating controls into data it can understand
    """
    def __init__(self, car_params):
        self.speed = 0 # -1 = max speed back, 0 = stop, 1 = max speed forward
        self.direction = 0 # -1 = left, 0 = straight, 1 = right
        self.state = 0
        self.car_params = car_params
        self.controller = self.find_mse_arduino()
        if not self.controller:
            rospy.logfatal('failed to initialized car hardware')
            exit(1)
        else:
            self.write_controls()
            self.state = 4
            self.calibrate()

    def __del__(self):
        self.write_controls(0, 0)
        self.controller.close()

    def find_mse_arduino(self):
        """checks if the arduino is connected
        and opens a connection if so
        """
        try:
            c = self.car_params
            serial_controller = serial.Serial(c['mse_path'], baudrate=c['baudrate'], timeout=c['timeout'])
            for _ in xrange(100):
                if 'mse' in ser.readline():
                    return serial_controller
            serial_controller.close()
        except:
            pass
        rospy.logfatal('could not find mse arduino (the one that controls the motors and steering)')
        return False

    def calibrate(self):
        """reads the arduino input from the handheld
        remote control and determines the null, max, and min
        values for the accelerator and steering wheel
        """
        self.remote_direction_null = False
        self.remote_direction_min = False
        self.remote_direction_max = False
        self.remote_speed_null = False
        self.remote_speed_min = False
        self.remote_speed_max = False

        pass

    def pwm_from_speed(self, speed):
        """translates the speed (float from -1 to 1)
        into a valid pwm value based on the car params
        """
        return int(min(params.speed_pwm_max, max(params.speed_pwm_min, (params.speed_pwm_null + params.speed_pwm_gain * speed))))

    def pwm_from_direction(self, direction):
        """translates the direction (float from -1 to 1)
        into a valid pwm value based on the car params
        """
        return int(min(params.direction_pwm_max, max(params.direction_pwm_min, (params.direction_pwm_null + params.direction_pwm_gain * direction))))

    def write_controls(self, speed=0, direction=0):
        """sends speed and direction data to the arduino
        format is '(<pwm direction>, <pwm speed + 10000>)'
        """
        if self.controller and self.state in [0, 1, 2, 3, 6, 9]:
            pwm_speed = self.pwm_from_speed(speed)
            pwm_direction = self.pwm_from_direction(direction)
            self.controller.write('(' + str(pwm_direction) + ',' + str(pwm_speed + 10000) + ')')

car = Car(params.car_info[params.current_car])

def controls_callback(controls_msg):
    """ros callback for published controls
    """
    car.speed = controls_msg.speed
    car.direction = controls_msg.direction
    car.write_controls(speed, direction)

def main():
    rospy.logdebug('starting bdd car node')
    rospy.init_node('bdd_car')
    rospy.Subscriber('bdd/controls/car_controls', BDDMsg.BDDControlsMsg, callback=controls_callback)
    rospy.spin()
    rospy.logdebug('exiting bdd car node')

if __name__ == '__main__':
    main()
