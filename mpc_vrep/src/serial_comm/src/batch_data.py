#!/usr/bin/env python
import rospy
import pandas as pd
import numpy as np
import csv
from std_msgs.msg import Float32MultiArray
import time

joint_handles = None
clientID = None
c = 100
counter = 0
end_eff_batch = np.array([])

pub = rospy.Publisher("/batched_value", Float32MultiArray, queue_size = 1)

def callback(end_effector):
    global joint_handles
    global clientID
    global counter
    global end_eff_batch
    global c
    # if c>3:    
    if counter < 5:
        e1 = end_effector.data
        end_eff = np.array(e1)
        end_eff_batch = np.append(end_eff_batch, end_eff) 
        counter += 1 
    else:
        counter = 0
        # c=0
        end_eff_data_batch = Float32MultiArray()
        end_eff_data_batch.data = end_eff_batch
        print(end_eff_batch)
        pub.publish(end_eff_data_batch)
        end_eff_batch = np.array([])
    # else:
    #     e = end_effector.data
    #     c+=1
    #     print("skipping")

def listener():
    global joint_handles
    global clientID
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("/end_effector", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
