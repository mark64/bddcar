#!/bin/sh

scp -P 12222 -r /home/mark/newdeepdrive/src mark@localhost:~/autocomm/
ssh -p 12222 mark@localhost "source /opt/ros/lunar/setup.bash && cd autocomm && catkin_make"
