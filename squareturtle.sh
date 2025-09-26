#!/bin/bash

# first publishing moving forward
for i in {1..12}
do
    echo "Turtle moves forward"
    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"

    # sleep for 2s to move forward for 2s

    echo  "Turtle turns 90 degrees"
    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.55}}"
    #sleep for 1s to let turtle rotate
done
echo "stopping the turtle after going around the square three times"