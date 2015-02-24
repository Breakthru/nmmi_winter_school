#!/usr/bin/env python
#
# Calculate the positition error in the VSA cubes
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


import rospy
from iai_qb_cube_msgs.msg import CubeStateArray
from iai_qb_cube_msgs.msg import CubeCmdArray
from iai_qb_cube_msgs.msg import CubeDataArray
from iai_qb_cube_msgs.msg import CubeData


joint_data_cmd = {'arm_1_joint':0, 'arm_2_joint':0, 'arm_3_joint':0}
joint_data_pos = {'arm_1_joint':0, 'arm_2_joint':0, 'arm_3_joint':0}
result_map = {'arm_1_joint':0, 'arm_2_joint':0, 'arm_3_joint':0}

cmd_fresh = False
pos_fresh = False

def cb_cube_commands(msg):
    #print msg
    for cmd in msg.commands:
        if joint_data_cmd.has_key(cmd.joint_name):
            joint_data_cmd[cmd.joint_name] = cmd.equilibrium_point
            global cmd_fresh
            cmd_fresh = True
        
def cb_cube_states(msg):
    #print msg
    for state in msg.states:
        if joint_data_pos.has_key(state.joint_name):
            joint_data_pos[state.joint_name] = state.pos_joint
            global pos_fresh
            pos_fresh = True


def publish_data(publisher):
    
    global pos_fresh, cmd_fresh
    
    data_points = []
    contact_scalar = 0
    
    if (not pos_fresh) and (not cmd_fresh):
        return
    
    for name in joint_data_cmd:
        data = CubeData()
        data.joint_name = name
        data.position_error = abs(joint_data_cmd[name] - joint_data_pos[name])
        #print("joint_cmd = %f  joint_pos = %f" %(joint_data_cmd[name], joint_data_pos[name]))
        contact_scalar += data.position_error
        data_points.append(data)
        
    cmd_fresh = False
    pos_fresh = False
    
    msg = CubeDataArray()
    msg.data = data_points
    msg.contact_scalar = contact_scalar
    
    publisher.publish(msg)

def main():

    rospy.init_node("arm_pos_error")
    rospy.loginfo("Coming up")
    
    sub_cmd = rospy.Subscriber("/iai_qb_cube_driver/command", CubeCmdArray, cb_cube_commands, queue_size=1)
    
    sub_state = rospy.Subscriber("/iai_qb_cube_driver/cube_state", CubeStateArray, cb_cube_states, queue_size=1)
    
    pub_data = rospy.Publisher("arm_pos_error/data", CubeDataArray, queue_size=3)

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        publish_data(pub_data)
        r.sleep()





if __name__ == "__main__":
    main()
    