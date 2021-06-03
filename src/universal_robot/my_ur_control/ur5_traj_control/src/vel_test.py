#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import Float64MultiArray
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
	
#global k
#to talk to virtual model
def GoalSend(data):
	data=data.data[6:12]
	print data
	global joint_pos
	global theta
	global jvel1
	global jvel2
	global jvel

	
	rate = rospy.Rate(125) # 10hz
	hello_str = JointTrajectory()
	hello_str.header = Header()
	hello_str.joint_names=JOINT_NAMES
	joint_states=rospy.wait_for_message("joint_states", JointState)
	jvel=joint_states.position[0]
	
	
	hello_str.points=[
				JointTrajectoryPoint(velocities=data, time_from_start=rospy.Duration(0.0)),
				JointTrajectoryPoint(time_from_start=rospy.Duration(0.0))]
		

	
	pub.publish(hello_str)
		
		#print k
		
if __name__ == '__main__':
	global client

try:
	rospy.init_node("test_move", anonymous=True, disable_signals=True)

	rospy.Subscriber("/solutions", Float64MultiArray, GoalSend)
	rospy.spin()

except KeyboardInterrupt:
	rospy.signal_shutdown("KeyboardInterrupt")
	raise
