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

from sensor_msgs.msg import JointState
from iai_qb_cube_msgs.msg import CubeCmdArray
from iai_qb_cube_msgs.msg import CubeCmd
from iai_qb_cube_msgs.msg import CubeStiff
from iai_qb_cube_msgs.msg import CubeStiffArray




class arm_ik_controller(object):
    def __init__(self):
        rospy.loginfo("Starting up")
        self.configured = False
        self.ros_transform_topic_name = "/arm_controller/command"
        self.ros_stiff_topic_name = "/arm_controller/stiff_command"
        self.tf_base_link_name = 'base_link_zero'
        self.tf_end_link_name = 'arm_fixed_finger'        
        
    def __del__(self):
        #Stop all the motors?
        rospy.loginfo("Exiting")
        
        
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
        self.arm_joint_names = self.kdl_kin.get_joint_names()
        
        rospy.loginfo("The joint names controlled will be:")
        rospy.loginfo("%s"%self.arm_joint_names)

        #Data structure containing the goals for the cubes
        self.arm_setpoints = dict()
        self.arm_jointdata = dict()
        for name in self.arm_joint_names:
            self.arm_setpoints[name] = {'stiff':0.0, 'eq_point':0.0}
            self.arm_jointdata[name] = {'pos':0.0}
            
        #Start the subscriber
        rospy.Subscriber(self.ros_transform_topic_name, geometry_msgs.msg.TransformStamped, self.cb_command, queue_size=1)
        
        #Another subscriber for the stiffness data
        rospy.Subscriber(self.ros_stiff_topic_name, CubeStiffArray, self.cb_stiff_command)
        
        #Subscriber to find the current joint positions
        rospy.Subscriber("/iai_qb_cube_driver/joint_state", JointState, self.cb_joint_states )
        
        #Prepare our publisher
        self.cubes_pub = rospy.Publisher("/iai_qb_cube_driver/command", CubeCmdArray, queue_size=3)
        
        
        
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
        
    def cb_stiff_command(self, msg):
        rospy.logdebug("Received new stiffness presets command")
        
        for cmd in msg.stiffness_presets:
            self.arm_setpoints[cmd.joint_name]['stiff'] = cmd.stiffness_preset
            rospy.loginfo("New stiffness, new goals: %s", self.arm_setpoints)
            
        self.send_command_to_cubes()
            
    def send_command_to_cubes(self):
        rospy.loginfo("Sending new command to cubes")

        #Prepare the message for the cube driver        
        out_msg = CubeCmdArray()
    
        #Fill in the values from the arm_setpoints dict  
        for joint_name in self.arm_joint_names:
            cubecmd = CubeCmd()
            cubecmd.joint_name = joint_name
            cubecmd.equilibrium_point = self.arm_setpoints[joint_name]['eq_point']
            cubecmd.stiffness_preset = self.arm_setpoints[joint_name]['stiff']  
            out_msg.commands.append(cubecmd)
    
        print out_msg
        self.cubes_pub.publish(out_msg)

    def get_current_joint_pos(self):
        q_current = []
        for name in self.arm_joint_names:
            q_current.append(self.arm_jointdata[name]['pos'])
        return q_current
    

    def cb_command(self, msg):
        rospy.loginfo("New command. source_frame: %s  tip_frame: %s", msg.header.frame_id, msg.child_frame_id)


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
        #Use TF for the transformation
        try:
            ps_in_base_link = self.tf_listener.transformPose(self.kdl_kin.base_link, ps)
        except:
            rospy.logerr("Could not transform the goal pose.")
            return
        

        #Do the inverse kinematics
        rospy.loginfo("Current joint pos: %s", self.get_current_joint_pos())
        q_sol = self.kdl_kin.inverse(ps_in_base_link, q_guess=self.get_current_joint_pos())
        
        if q_sol is None:
            rospy.loginfo("Looking for IK from a random start position:w"
                          "")
            q_sol = self.kdl_kin.inverse_search(ps_in_base_link, 0.1)


        if q_sol is None:
            rospy.loginfo("IK did not find a solution")
            return


        #Save the result to the arm_setpoints dict
        for i,name in enumerate(self.arm_joint_names):
            self.arm_setpoints[name]['eq_point'] = q_sol[i]

        self.send_command_to_cubes()
        

    def cb_joint_states(self, msg):
        
        for i, name in enumerate(msg.name):
            if self.arm_jointdata.has_key(name):
                self.arm_jointdata[name]['pos'] = msg.position[i]
            




#if False:
    #robot = URDF.from_parameter_server()
    #tree = kdl_tree_from_urdf_model(robot)
    #print tree.getNrOfSegments()
    #chain = tree.getChain(base_link_name, end_link_name)
    #print chain.getNrOfJoints()
    
    #kdl_kin = KDLKinematics(robot, base_link_name, end_link_name)
    #q = kdl_kin.random_joint_angles()
    #pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
    #q_ik = kdl_kin.inverse(pose, q + 0.3) # inverse kinematics
    #if q_ik is not None:
        #pose_sol = kdl_kin.forward(q_ik) # should equal pose
    
    #J = kdl_kin.jacobian(q)
    #print 'q:', q
    #print 'q_ik:', q_ik
    #print 'pose:', pose
    #if q_ik is not None:
        #print 'pose_sol:', pose_sol
    #print 'J:', J
    



def main():
    change_ps_name("arm_ik_controller.py")
    
    armc = arm_ik_controller()
    
    if not armc.configure():
        rospy.logerr("Could not configure the arm_ik_controller. Exiting")
        return
    
    
    
    
    #(pos, quat) = armc.get_tf_pose('base_link_zero', 'right_holder_link')
    #print pos
    #print "quat:", quat
    
    import time
    while not rospy.is_shutdown():
        time.sleep(0.05)
    
    


if __name__ == "__main__":
    main()
    