#!/usr/bin/env python

import rospy
from hsrb_interface import Robot, exceptions, geometry
from std_srvs.srv import Empty
import math
rospy.loginfo('Getting robot resources')
print ('Starting')
robot = None
while not rospy.is_shutdown():
    try:
        robot = Robot()
        whole_body = robot.try_get('whole_body')
        omni_base = robot.try_get('omni_base')
        print ('got resources')
        break
    except (exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
        rospy.logerr_throttle(1, 'Failed to obtain resource: {}\nRetrying...'.format(e))

whole_body.gaze_point(point=geometry.Vector3(x=5, y=5, z=0), ref_frame_id='map')

rospy.sleep(5)

