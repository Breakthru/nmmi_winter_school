#!/bin/sh

rostopic pub /arm_controller/command geometry_msgs/TransformStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base_link_zero'
child_frame_id: 'arm_fixed_finger'
transform:
  translation:
    x: 0.074
    y: 0.16
    z: 0.0
  rotation:
    x: 0.707
    y: 0.707
    z: 0.0
    w: 0.0" 


