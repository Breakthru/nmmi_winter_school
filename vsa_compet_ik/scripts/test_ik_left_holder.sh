#!/bin/sh

rostopic pub -1 /arm_controller/command geometry_msgs/TransformStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'left_holder_link'
child_frame_id: 'arm_fixed_finger'
transform:
  translation:
    x: $1
    y: $2
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 


