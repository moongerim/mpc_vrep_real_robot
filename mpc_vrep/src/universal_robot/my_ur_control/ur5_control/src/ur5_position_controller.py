#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs


#Define a UR joint positions publisher for joint controllers.
def ur_joint_positions_publisher(jointCmd):

	#Initiate node for controlling joint positions.
        rospy.init_node('ur5_joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/ur5/shoulder_pan_joint_position_controller/command', Float64, queue_size=1)
	pub2 = rospy.Publisher('/ur5/shoulder_lift_joint_position_controller/command', Float64, queue_size=1)
	pub3 = rospy.Publisher('/ur5/elbow_joint_position_controller/command', Float64, queue_size=1)
	pub4 = rospy.Publisher('/ur5/wrist_1_joint_position_controller/command', Float64, queue_size=1)
        pub5 = rospy.Publisher('/ur5/wrist_2_joint_position_controller/command', Float64, queue_size=1)
	pub6 = rospy.Publisher('/ur5/wrist_3_joint_position_controller/command', Float64, queue_size=1)

	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	#while not rospy.is_shutdown():
        while (i<50):
	  pub1.publish(jointCmd[0])
	  pub2.publish(jointCmd[1])
	  pub3.publish(jointCmd[2])
	  pub4.publish(jointCmd[3])
	  pub5.publish(jointCmd[4])
	  pub6.publish(jointCmd[5])

	  i = i+1 #increment i

	  rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: 
          rospy.sleep(1) 
          ur_joint_positions_publisher([-1.9628, -1.5602, 1.4466, 0.0, 1.2545, -3.0039])
          

        except rospy.ROSInterruptException: pass




