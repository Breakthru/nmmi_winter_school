#!/usr/bin/env python

import PyKDL as kdl

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

base_link_name = 'base_link_zero'
end_link_name = '3dof_arm_fixed_finger'

robot = URDF.from_parameter_server()
tree = kdl_tree_from_urdf_model(robot)
print tree.getNrOfSegments()
chain = tree.getChain(base_link_name, end_link_name)
print chain.getNrOfJoints()

kdl_kin = KDLKinematics(robot, base_link_name, end_link_name)
q = kdl_kin.random_joint_angles()
pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
q_ik = kdl_kin.inverse(pose, q+0.3) # inverse kinematics
if q_ik is not None:
  pose_sol = kdl_kin.forward(q_ik) # should equal pose

J = kdl_kin.jacobian(q)
print 'q:', q
print 'q_ik:', q_ik
print 'pose:', pose
if q_ik is not None:
  print 'pose_sol:', pose_sol
print 'J:', J


