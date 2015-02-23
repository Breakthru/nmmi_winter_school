#!/usr/bin/env python
#
# Node to control the VSA_3DOF_arm given a TransformStamped from the executive
#
# Copyright (c) 2015 Alexis Maldonado Herrera <amaldo at cs.uni-bremen.de>
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

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from hrl_geom.pose_converter import PoseConv

def transformStamped_to_kdlFrame(ts):
    frame = kdl.Frame()
    frame.p = kdl.Vector(ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z)
    frame.M = kdl.Rotation.Quaternion(ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w)
    
    return([ts.header.frame_id, frame])

def kdlFrame_to_transformStamped(frame, frame_id='base_link_zero', child_frame_id='arm_fixed_finger'):
    ts = TransformStamped()
    ts.transform.translation.x = frame.p.x()
    ts.transform.translation.y = frame.p.y()
    ts.transform.translation.z = frame.p.z()
    
    quat = frame.M.GetQuaternion()
    ts.transform.rotation.x = quat[0]
    ts.transform.rotation.y = quat[1]
    ts.transform.rotation.z = quat[2]
    ts.transform.rotation.w = quat[3]
    
    ts.header.frame_id = frame_id
    ts.child_frame_id = child_frame_id

    return(ps)
    

def poseStamped_to_kdlFrame(ps):
    frame = kdl.Frame()

    frame.p = kdl.Vector(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
    frame.M = kdl.Rotation.Quaternion(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w)
    
    return([ps.header.frame_id, frame])

def kdlFrame_to_poseStamped(frame, frame_id='base_link_zero'):
    ps = PoseStamped()
    ps.pose.position.x = frame.p.x()
    ps.pose.position.y = frame.p.y()
    ps.pose.position.z = frame.p.z()
    
    quat = frame.M.GetQuaternion()
    ps.pose.orientation.x = quat[0]
    ps.pose.orientation.y = quat[1]
    ps.pose.orientation.z = quat[2]
    ps.pose.orientation.w = quat[3]
    ps.header.frame_id = frame_id
    
    return(ps)
    

class MiniInterpolator(object):
    def __init__(self):
        rospy.loginfo("MiniInterpolator starting up")
    def configure(self):
        self.dst_pose = kdl.Frame()
        self.current_pose = kdl.Frame()
        self.trajectory = None
        self.start_time = rospy.Time.now()
        
        
    def set_current_pose(self, frame):
        self.current_pose = frame
        
    def new_goal(self, frame):
        self.dst_pose = frame
        
    def replan_movement(self):

        #find distance
        dist = kdl.diff(self.current_pose, self.dst_pose)
        
        #make 10 points
        
        dt = 0.1
        self.start_time = rospy.Time.now()  
        self.pose_vector = []
        for i in range(10 + 1):
            inter_pose = kdl.addDelta(self.current_pose, dist, dt * i)
            inter_time = self.start_time + rospy.Duration(dt) * i
            self.pose_vector.append([inter_time, inter_pose])
        
        
    def query_plan(self):
        
        for time, pose in self.pose_vector:
            if time > rospy.Time.now():
                #Found the current setpoint
                return(pose)
    
        #If we get here, found no unreached setpoint, so just return the last goal frame
        return(self.dst_pose)
    
    

class MiniVectorField(object):
    def __init__(self ):
        rospy.loginfo("MiniVectorField starting up")
        
        
    def configure(self, tf_base_link_name, tf_end_link_name):
        self.tf_base_link_name = tf_base_link_name
        self.tf_end_link_name = tf_end_link_name

        #Variables holding the joint information
        self.robot = URDF.from_parameter_server()
        self.tree = kdl_tree_from_urdf_model(self.robot)       
        self.chain = self.tree.getChain(self.tf_base_link_name, self.tf_end_link_name)
        self.kdl_kin = KDLKinematics(self.robot, self.tf_base_link_name, self.tf_end_link_name)   

        self.arm_joint_names = self.kdl_kin.get_joint_names()        
        
        self.jnt_pos = kdl.JntArray(self.chain.getNrOfJoints())
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ikv_solver = kdl.ChainIkSolverVel_wdls(self.chain)
        self.ikv_solver.setLambda(0.0001)
        #self.ikv_solver.setWeightTS([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,0,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        
        self.ik_solver = kdl.ChainIkSolverPos_NR(self.chain,
                                             self.fk_solver,
                                             self.ikv_solver)  
        
    def get_frame(self):
        frame = kdl.Frame()
        self.fk_solver.JntToCart(self.jnt_pos, frame)
        return frame
    
    def try_move(self, frame):
        #initialize joint values for search to current angles
        q_out = kdl.JntArray(self.jnt_pos)
    
        delta_q = kdl.JntArray(self.jnt_pos.rows())
    
        maxiter = 300
        eps = 0.002
        f = Frame()
        searching = True
    
        iter = 0
        while(searching):
            iter += 1
    
            #forward kin of the current joints
            self.fk_solver.JntToCart(q_out, f);
            #distance in 6DOF between goal and current frame
            delta_twist = kdl.diff(f, frame);
            
            #velocity inv kinematics
            self.ikv_solver.CartToJnt(q_out, delta_twist, delta_q);
            
            #Add delta angle
            Add(q_out, delta_q, q_out);
    
            #Algorithm has converged:

            print "Delta_twist:"
            print delta_twist
            
        
            if ((abs(delta_twist.vel[0]) < eps) and (abs(delta_twist.vel[1]) < eps) and (abs(delta_twist.vel[2]) <eps )):
                print("found a solution")
                searching = False
                self.jnt_pos = q_out
                return(True)
                
            if (iter > maxiter):
                print "IK: Did not converge"
                return False

        
        
    
    def set_cart_goal_from_ps(self, ps):
        
        goal_frame = kdl.Frame()
        
        if ps1.header.frame_id != self.tf_base_link_name:
            rospy.logwarn("MiniVectorField: The frame_id does not match the expected one")
            return
        
        goal_frame.p = kdl.Vector(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
        goal_frame.M = kdl.Rotation.Quaternion(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w)
        
        
        

        
    def set_cart_goal(self, cart_goal):
        self.cart_goal = None

class Arm_ik_controller(object):
    def __init__(self):
        rospy.loginfo("Starting up")
        self.configured = False
        self.ros_transform_topic_name = "/arm_controller/command"
        self.ros_stiff_topic_name = "/arm_controller/stiff_command"
        self.tf_base_link_name = 'base_link_zero'
        self.tf_end_link_name = 'arm_fixed_finger'        
        #self.minivf = MiniVectorField()
        self.interpolator = MiniInterpolator()
        
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
        #self.tree = kdl_tree_from_urdf_model(self.robot)       
        #self.chain = self.tree.getChain(self.tf_base_link_name, self.tf_end_link_name)
        self.kdl_kin = KDLKinematics(self.robot, self.tf_base_link_name, self.tf_end_link_name)    
        
        #Variables holding the joint information
        self.arm_joint_names = self.kdl_kin.get_joint_names()
        
        rospy.loginfo("The joint names controlled will be:")
        rospy.loginfo("%s"%self.arm_joint_names)

        #Mini Vector field
        #self.minivf.configure(self.tf_base_link_name, self.tf_end_link_name)

        #Data structure containing the goals for the cubes
        self.arm_setpoints = dict()
        self.arm_jointdata = dict()
        for name in self.arm_joint_names:
            self.arm_setpoints[name] = {'stiff':0.0, 'eq_point':0.0}
            self.arm_jointdata[name] = {'pos':0.0}
            
            
        self.cart_goal = None
        self.fresh_cart_goal = False
        
        #Start the subscriber
        rospy.Subscriber(self.ros_transform_topic_name, geometry_msgs.msg.TransformStamped, self.cb_command, queue_size=1)
        
        #Another subscriber for the stiffness data
        rospy.Subscriber(self.ros_stiff_topic_name, CubeStiffArray, self.cb_stiff_command)
        
        #Subscriber to find the current joint positions
        rospy.Subscriber("/iai_qb_cube_driver/joint_state", JointState, self.cb_joint_states )
        
        #Prepare our publisher
        self.cubes_pub = rospy.Publisher("/iai_qb_cube_driver/command", CubeCmdArray, queue_size=3)
        
        #Set up a timer for the function talking to the cubes
        rospy.Timer(rospy.Duration(0.01), self.cb_cube_controller)
        
        
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
        rospy.logdebug("Sending new command to cubes")

        #Prepare the message for the cube driver        
        out_msg = CubeCmdArray()
    
        #Fill in the values from the arm_setpoints dict  
        for joint_name in self.arm_joint_names:
            cubecmd = CubeCmd()
            cubecmd.joint_name = joint_name
            cubecmd.equilibrium_point = self.arm_setpoints[joint_name]['eq_point']
            cubecmd.stiffness_preset = self.arm_setpoints[joint_name]['stiff']  
            out_msg.commands.append(cubecmd)
        self.cubes_pub.publish(out_msg)

    def get_current_joint_pos(self):
        q_current = []
        for name in self.arm_joint_names:
            q_current.append(self.arm_jointdata[name]['pos'])
        return q_current
    

    def cb_command(self, msg):
        '''Receives the new goal frame as a TransformStamped'''
        rospy.logdebug("New command. source_frame: %s  tip_frame: %s", msg.header.frame_id, msg.child_frame_id)


        if msg.child_frame_id != self.tf_end_link_name:
            rospy.logerr("Mismatch in end link frames. Got: %s  Wanted: %s", msg.child_frame_id, self.kdl_kin.end_link)        
            return
        
        #FIXME: react to changes in tf_end_link_name and rebuild the chain accordingly

 
        #Convert into a PS
        from geometry_msgs.msg import PoseStamped
        ps = PoseStamped()
        ps.pose.position = msg.transform.translation
        ps.pose.orientation = msg.transform.rotation
        ps.header.frame_id = msg.header.frame_id
        

        #Save the new desired cartesian pose
        self.cart_goal = poseStamped_to_kdlFrame(ps)
        self.fresh_cart_goal = True
        
        
        

    def cb_cube_controller(self, event):
        '''Function driven by a timer that communicates with the cubes'''
        
        if self.cart_goal is None:
            return
        
        [frame_id, kdl_frame] = self.cart_goal
        #print self.get_current_joint_pos()
        #return
    
        current_homo_mat = self.kdl_kin.forward(self.get_current_joint_pos())
        (pos, quat) = PoseConv.to_pos_quat(current_homo_mat)
        current_kdl_frame = kdl.Frame()
        current_kdl_frame.p = kdl.Vector(pos[0], pos[1], pos[2])
        current_kdl_frame.M = kdl.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
        
        
        if self.fresh_cart_goal:
            self.interpolator.new_goal(kdl_frame)
            self.interpolator.set_current_pose(current_kdl_frame)
            self.interpolator.replan_movement()
            
        kdl_frame = self.interpolator.query_plan()
        
        
        [frame_id, kdl_frame_final_goal] = self.cart_goal
        
        ps = kdlFrame_to_poseStamped(kdl_frame, frame_id)
        
        #Use TF for the transformation
        try:
            ps_in_base_link = self.tf_listener.transformPose(self.kdl_kin.base_link, ps)
        except:
            print ps
            rospy.logerr("Could not transform the goal pose.")
            return
    
        #Do the inverse kinematics
        rospy.logdebug("Current joint pos: %s", self.get_current_joint_pos())
        q_sol = self.kdl_kin.inverse(ps_in_base_link, q_guess=self.get_current_joint_pos())
    
        if q_sol is None:
            rospy.loginfo("Looking for IK from a random start position")
            q_sol = self.kdl_kin.inverse_search(ps_in_base_link, 0.1)
    
    
        if q_sol is None:
            rospy.loginfo("IK did not find a solution")
            return
    
    
        #Save the result to the arm_setpoints dict
        for i,name in enumerate(self.arm_joint_names):
            self.arm_setpoints[name]['eq_point'] = q_sol[i]
    
        self.send_command_to_cubes()


        

    def cb_joint_states(self, msg):
        '''Store the current joint positions on a variable to use in the ik calculations'''
        
        for i, name in enumerate(msg.name):
            if self.arm_jointdata.has_key(name):
                self.arm_jointdata[name]['pos'] = msg.position[i]
            


    


# -------------------- main()  ---------------------------
def main():
    change_ps_name("arm_ik_controller.py")
    
    armc = Arm_ik_controller()
    
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
    