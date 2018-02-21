#!/usr/bin/env python3
import rospy
import bdd.msg as amsg
from bdd_driver import camera, ml
Camera = camera.Camera
ML = ml.ML
from params import params

def main():
    print('starting bdd driver node')
    rospy.init_node('bdd_driver')
    controls_pub = rospy.Publisher('bdd_controls', amsg.AutocommControlsMsg, queue_size=1)
    cam = Camera(test_mode=params.test_mode)
    models = []
    for model_info in params.models:
        models,append(ML(model_info))
    models = [ML(params.weights_path) for _ in range(params.num_models)]
    if not cam.initialized and not params.test_mode:
        print('failed to connect to zed camera..exiting')
        exit(1)
    rate = rospy.Rate(cam.camera_fps)
    while not rospy.is_shutdown():
        if cam.capture():
            controls = [model.output_controls(cam.left, cam.right) for model in models][0]
            control = ml.aggregate_controls(controls)
            msg = amsg.BDDControlsMsg(speed=control.speed, direction=control.direction)
            controls_pub.publish(msg)
        rate.sleep()
    print('bdd driver node exiting')

if __name__ == '__main__':
    main()
