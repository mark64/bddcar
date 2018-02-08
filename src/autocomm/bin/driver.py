#!/usr/bin/env python3
import rospy
import autocomm.msg as amsg
from autocomm_driver import camera, ml
Camera = camera.Camera
ML = ml.ML
from autocomm_params import params

def main():
    print('starting autocomm driver node')
    rospy.init_node('autocomm_driver')
    controls_pub = rospy.Publisher('autocomm_controls', amsg.AutocommControlsMsg, queue_size=1)
    cam = Camera(test_mode=params.test_mode)
    models = [ML(params.weights_path) for _ in range(params.num_models)]
    if not cam.initialized and not params.test_mode:
        print('failed to connect to zed camera..exiting')
        exit(1)
    while not rospy.is_shutdown():
        if cam.capture():
            controls = [model.output_controls(cam.left, cam.right) for model in models][0]
            msg = amsg.AutocommControlsMsg(speed=controls.speed, direction=controls.direction)
            controls_pub.publish(msg)
    print('autocomm driver node exiting')

if __name__ == '__main__':
    main()
