#!/usr/bin/env python
#
# Node to feed the interactive marker updates to the arm_ik_controller - NMMI Winter school
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


import rospy
import sys
import geometry_msgs

from visualization_msgs.msg import InteractiveMarkerUpdate
from geometry_msgs.msg import TransformStamped

class DataSender(object):
    def __init__(self):
        rospy.loginfo("Coming up")
        
    def configure(self):
        rospy.init_node("int_marker_to_arm_ik", argv=sys.argv, anonymous=False) 
        self.marker_sub = rospy.Subscriber("/arm_int_markers/update", InteractiveMarkerUpdate, self.int_marker_update_cb, queue_size=3)
        self.transform_pub = rospy.Publisher("/arm_controller/command", TransformStamped, queue_size=1)

    def int_marker_update_cb(self, msg):
        rospy.logdebug("Got a new update")
        
        if len(msg.poses) > 0:
            tr_msg = TransformStamped()
            tr_msg.header.frame_id = msg.poses[0].header.frame_id
            
            tr_msg.transform.translation = msg.poses[0].pose.position
            tr_msg.transform.rotation = msg.poses[0].pose.orientation
            
            tr_msg.child_frame_id = 'arm_fixed_finger'
            
            self.transform_pub.publish(tr_msg)
        


def main():
    
    
    sender = DataSender()
    sender.configure()
    
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        r.sleep()    
    
    

if __name__ == "__main__":
    main()