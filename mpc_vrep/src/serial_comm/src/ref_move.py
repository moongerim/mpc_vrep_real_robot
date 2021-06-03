#!/usr/bin/env python
import rospy
from serial_comm.msg import Pos_Rot
from std_msgs.msg import Float32MultiArray
from pyquaternion import Quaternion

from gazebo_msgs.msg import ModelState , ModelStates
pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=3)


class Server:
    def __init__(self):
        self.axes = None
        self.orientation =[0,0,0]
            

    def updateModel(self,msg):
        new_pos=ModelState()
        new_pos.model_name='unit_box'
        new_pos.pose.position.x=msg.data[0]
        new_pos.pose.position.y=msg.data[1]
        new_pos.pose.position.z=msg.data[2]

        quat = Quaternion(msg.data[3], msg.data[4], msg.data[5], msg.data[6])
        rot = Quaternion(1, 0.0, 0.0, 0.0)
        quat = rot*quat*rot.inverse
        elements = quat.elements

        new_pos.pose.orientation.w = elements[0]
        new_pos.pose.orientation.x = elements[1]
        new_pos.pose.orientation.y = elements[2]
        new_pos.pose.orientation.z = elements[3]
        pub.publish(new_pos)


if __name__ == '__main__':
    rospy.init_node('model_control')
    server = Server()
    rospy.Subscriber("/end_effector", Float32MultiArray, server.updateModel)
    rospy.spin()
