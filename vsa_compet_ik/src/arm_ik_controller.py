#!/usr/bin/env python
#
# Node to control the VSA_3DOF_arm given a TransformStamped from the executive
#
# Copyright (c) 2009-2015 Alexis Maldonado Herrera <amaldo at cs.uni-bremen.de>
# Universitaet Bremen, Institute for Artificial Intelligence (AGKI).
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.


def change_ps_name(name):
    """Change process name as used by ps(1) and killall(1)."""
    try:
        import ctypes

        libc = ctypes.CDLL('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    except:
        pass

def kdlpose_to_matrix(pose):
    p = pose.p
    M = pose.M
    return numpy.mat([[M[0,0], M[0,1], M[0,2], p.x()], 
                   [M[1,0], M[1,1], M[1,2], p.y()], 
                   [M[2,0], M[2,1], M[2,2], p.z()],
                   [     0,      0,      0,     1]])


import rospy
import sys
import PyKDL as kdl
import tf
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy
import geometry_msgs

from iai_qb_cube_msgs.msg import CubeCmdArray
from iai_qb_cube_msgs.msg import CubeCmd



class arm_ik_controller(object):
    def __init__(self):
        rospy.loginfo("Starting up")
        self.configured = False
        self.ros_transform_topic_name = "/arm_controller/command"
        self.tf_base_link_name = 'base_link_zero'
        self.tf_end_link_name = 'arm_fixed_finger'        
        
    def __del__(self):
        #Stop all the motors?
        rospy.loginfo("Exiting")
        
    def cb_command(self, msg):
        rospy.loginfo("Got a new command")
        rospy.loginfo("source_frame: %s  tip_frame: %s", msg.header.frame_id, msg.child_frame_id)
        
        
        debug = False
        
        if debug: 
            pose = kdl.Frame()
            
            pose.p = kdl.Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
            pose.M = kdl.Rotation.Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w)
            
            print "trans: ", pose.p
            print "quat: ", pose.M.GetQuaternion()
           
       
        if msg.child_frame_id != self.tf_end_link_name:
            rospy.logerr("Mismatch in end link frames. Got: %s  Wanted: %s", msg.child_frame_id, self.kdl_kin.end_link)        
            return
        #FIXME: react to changes in tf_end_link_name and rebuild the chain accordingly
        
        #Transform the desired pose to the right reference frame
        from geometry_msgs.msg import PoseStamped
        ps = PoseStamped()
        ps.pose.position = msg.transform.translation
        ps.pose.orientation = msg.transform.rotation
        ps.header.frame_id = msg.header.frame_id
        #Use TF for the transformation  FIXME: Guard this and react to not finding it
        ps_in_base_link = self.tf_listener.transformPose(self.kdl_kin.base_link, ps)
        
        #Do the inverse kinematicss
        q_sol = self.kdl_kin.inverse(ps_in_base_link, q_guess=None)
        
        
        if q_sol is None:
            rospy.loginfo("IK did not find a solution")
            return
            
        joint_names = self.kdl_kin.get_joint_names()
        
        
        #Prepare the message for the cube driver        
        out_msg = CubeCmdArray()
        
        for i,joint in enumerate(joint_names):
            cubecmd = CubeCmd()
            cubecmd.joint_name = joint
            cubecmd.equilibrium_point = q_sol[i]
            cubecmd.stiffness_preset = 0  #FIXME
            out_msg.commands.append(cubecmd)
            
        print out_msg
        self.cubes_pub.publish(out_msg)
        
        
        
    def configure(self):
        if self.configured:
            return False
        
        rospy.init_node("arm_ik_controller", argv=sys.argv, anonymous=False)
        self.tf_listener = tf.TransformListener()
        
        #Parse the URDF tree
        self.robot = URDF.from_parameter_server()
        self.tree = kdl_tree_from_urdf_model(self.robot)       
        self.chain = self.tree.getChain(self.tf_base_link_name, self.tf_end_link_name)
        self.kdl_kin = KDLKinematics(self.robot, self.tf_base_link_name, self.tf_end_link_name)    
        
        #Variables holding the joint information
        self.arm_joint_pos = numpy.array([0, 0, 0])
        self.arm_joint_names = self.kdl_kin.get_joint_names()
        
        rospy.loginfo("The joint names controlled will be:")
        rospy.loginfo("%s"%self.arm_joint_names)
        
        #Start the subscriber
        rospy.Subscriber(self.ros_transform_topic_name, geometry_msgs.msg.TransformStamped, self.cb_command )
        
        #Prepare our publisher
        self.cubes_pub = rospy.Publisher("/iai_qb_cube_driver/command", CubeCmdArray)
        
        
        
        return True
    
        
    def get_tf_pose(self, tf_parent, tf_child, timeout=2):
        '''Get the transformation from parent to child frame using tf'''
        time_of_transform = rospy.Time(0) # just get the latest
        try:
            self.tf_listener.waitForTransform(tf_parent, tf_child, time_of_transform, rospy.Time(timeout))
        except:
            rospy.logerr("Could not transform %s -> %s" %(tf_parent, tf_child))
            return None
        
        tr = self.tf_listener.lookupTransform(tf_parent, tf_child, time_of_transform)
        #return a tuple (pos, quat)
        return tr
        

if False:
    

    
    robot = URDF.from_parameter_server()
    tree = kdl_tree_from_urdf_model(robot)
    print tree.getNrOfSegments()
    chain = tree.getChain(base_link_name, end_link_name)
    print chain.getNrOfJoints()
    
    kdl_kin = KDLKinematics(robot, base_link_name, end_link_name)
    q = kdl_kin.random_joint_angles()
    pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
    q_ik = kdl_kin.inverse(pose, q + 0.3) # inverse kinematics
    if q_ik is not None:
        pose_sol = kdl_kin.forward(q_ik) # should equal pose
    
    J = kdl_kin.jacobian(q)
    print 'q:', q
    print 'q_ik:', q_ik
    print 'pose:', pose
    if q_ik is not None:
        print 'pose_sol:', pose_sol
    print 'J:', J
    



def main():
    change_ps_name("arm_ik_controller.py")
    
    armc = arm_ik_controller()
    
    if not armc.configure():
        rospy.logerr("Could not configure the arm_ik_controller. Exiting")
        return
    
    
    
    
    (pos, quat) = armc.get_tf_pose('base_link_zero', 'right_holder_link')
    print pos
    print "quat:", quat
    
    import time
    while not rospy.is_shutdown():
        time.sleep(0.05)
    
    


if __name__ == "__main__":
    main()
    