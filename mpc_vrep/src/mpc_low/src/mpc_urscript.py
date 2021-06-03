#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
high_pub = rospy.Publisher('/high_joint_states', JointState, queue_size=1)
low_pub = rospy.Publisher('/low_joint_states', JointState, queue_size=1)

def talker(data):
    vel = data.data[0:6]
    hello_str = "speedj(["+str(vel[0])+","+str(vel[1])+","+str(vel[2])+","+str(vel[3])+","+str(vel[4])+","+str(vel[5])+"],"+"5.0"+",0.1)" 
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

def callback(data):
    high_pub.publish(data)
    low_pub.publish(data)

def main():
    global velocity
    rospy.init_node("data_logging", anonymous=True)
    rospy.Subscriber("/MPC_solutions", Float64MultiArray, talker)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()
    print("exit")

if __name__ == '__main__': main()


