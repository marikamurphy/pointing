#!/usr/bin/env python

import rospy
from hsrb_interface import Robot, exceptions
from std_srvs.srv import Empty
import math
from groovy_motion import GroovyMotion #when this is pushed to Villa -> villa_helpers.groovy_motion
rospy.loginfo('Getting robot resources')
print ('Starting')
robot = None
while not rospy.is_shutdown():
    try:
        robot = Robot()
        whole_body = robot.try_get('whole_body')
        groovy_motion = GroovyMotion()
        omni_base = robot.try_get('omni_base')
        print ('got resources')
        break
    except (exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
        rospy.logerr_throttle(1, 'Failed to obtain resource: {}\nRetrying...'.format(e))
stop_head = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
start_head = rospy.ServiceProxy('/viewpoint_controller/start', Empty)

goals = {
    'arm_lift_joint': 0.3,
    'arm_flex_joint': -math.pi / 4,
    'arm_roll_joint': 0.0,
    'wrist_flex_joint': -1.57,
    'wrist_roll_joint': 0.0,
    'head_tilt_joint': -40 * math.pi / 180,
    'head_pan_joint': -math.pi
}
motion = groovy_motion.move_to_joint_positions(goals)
start_head()

# Wait on motion
motion(10., soft_wait=True)
stop_head()
rospy.sleep(5)

