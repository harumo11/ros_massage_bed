#!/usr/bin/env python

import rospy
import tm_msgs.srv

rospy.wait_for_service('/tm_driver/set_positions')
srv = rospy.ServiceProxy('/tm_driver/set_positions', tm_msgs.srv.SetPositions)
print(srv)
print(tm_msgs.srv.SetPositions)

# HERE!!!
# Change Target Robot arm hand position!!
#                  x      y       z      roll   pitch yaw
target_position = [0.536, -0.121, 0.456, -3.13, 0.0,  1.57]

res = srv.call(positions=target_position, velocity=0.4, acc_time=0.2,
               blend_percentage=10, fine_goal=False, motion_type=tm_msgs.srv.SetPositions._request_class.LINE_T)


print(res)
