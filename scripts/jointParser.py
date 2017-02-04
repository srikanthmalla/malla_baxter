#!/usr/bin/env python
import rospy
from malla_baxter import markers
import numpy as np
from vicon_bridge.msg import Markers
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher('/jointangles', Float32MultiArray, queue_size=10)


def callback(data):
    

    root = data.markers[0].translation;
    shoulder = data.markers[1].translation
    elbow = data.markers[2].translation
    v0 = np.array([root.x,root.y,root.z]) - np.array([shoulder.x,shoulder.y,shoulder.z])
    v1 = np.array([shoulder.x, shoulder.y, shoulder.z]) - np.array([elbow.x, elbow.y, elbow.z])
    
    cosang = np.dot(v0,v1)
    sinang = np.linalg.norm(np.cross(v0,v1))
    angle = np.arctan2(sinang,cosang)
    #angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
    #print angle
    # rospy.loginfo(rospy.get_caller_id() + "I heard %f", angle)
    f = Float32MultiArray()
    f.data = [angle]
    pub.publish(f)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)


    rospy.Subscriber("/vicon/markers", Markers, callback)

    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()