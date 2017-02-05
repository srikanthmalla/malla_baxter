#!/usr/bin/env python
import rospy
#from malla_baxter import markers
import numpy as np
from vicon_bridge.msg import Markers
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher('/jointangles', Float32MultiArray, queue_size=10)

#MultiArray as GLobal
F = Float32MultiArray()

#Initialize it to list -- No Need!
F.data = []

def callback(data):
    
    #Get All Data
    root = data.markers[0].translation;
    shoulder1 = data.markers[1].translation
    shoulder2 = data.markers[2].translation
    shoulder3 = data.markers[3].translation
    muscle = data.markers[4].translation
    elbow = data.markers[5].translation
    forearm = data.markers[6].translation
    wrist1 = data.markers[7].translation
    wrist2 = data.markers[8].translation
    wrist3 = data.markers[9].translation

    """
    3 axis are present in the shoulder, X-Y-Z ; Z = Vertical (opp gravity), X - Eye axis, Y- the remaining
    Rot. around X - Angle between Root - shoulder1 - muscle
    Rot. around Y - No Need
    Rot. aroutn Z - Angle between shoudler2 - shoulder1 - elbow
    """
    # Rotation around X
    v_root2shoulder1 = np.array([root.x,root.y,root.z]) - np.array([shoulder1.x,shoulder1.y,shoulder1.z])
    v_shoulder12muscle = np.array([shoulder1.x, shoulder1.y, shoulder1.z]) - np.array([muscle.x, muscle.y, muscle.z])
    cosang_x = np.dot(v_root2shoulder1,v_shoulder12muscle)
    sinang_x = np.linalg.norm(np.cross(v_root2shoulder1,v_shoulder12muscle))
    angle_x = np.arctan2(sinang_x,cosang_x)


    # #Rotation around Y
    # v_root2shoulder1 = np.array([root.x,root.y,root.z]) - np.array([shoulder1.x,shoulder1.y,shoulder1.z])
    # v_shoulder12muscle = np.array([shoulder1.x, shoulder1.y, shoulder1.z]) - np.array([muscle.x, muscle.y, muscle.z])
    # cosang_x = np.dot(v_root2shoulder1,v_shoulder12muscle)
    # sinang_x = np.linalg.norm(np.cross(v_root2shoulder1,v_shoulder12muscle))
    # angle_x = np.arctan2(sinang_x,cosang_x)


    # #Rotation around Z
    # v_shoulder22shoulder1 = np.array([shoulder2.x,shoulder2.y,shoulder2.z]) - np.array([shoulder1.x,shoulder1.y,shoulder1.z])
    # v_shoulder12elbow = np.array([elbow.x, elbow.y, elbow.z]) - np.array([elbow.x, elbow.y, elbow.z])
    # cosang_z = np.dot(v_shoulder22shoulder1,v_shoulder12elbow)
    # sinang_z = np.linalg.norm(np.cross(v_shoulder22shoulder1,v_shoulder12elbow))
    # angle_z = np.arctan2(sinang_z,cosang_z)

    #Rotation around Z
    v_shoulder22shoulder1 = np.array([shoulder2.x,shoulder2.y,shoulder2.z]) - np.array([shoulder1.x,shoulder1.y,shoulder1.z])
    v_shoulder12elbow = np.array([shoulder1.x, shoulder1.y, shoulder1.z]) - np.array([elbow.x, elbow.y, elbow.z])
    cosang_z = np.dot(v_shoulder22shoulder1,v_shoulder12elbow)
    sinang_z = np.linalg.norm(np.cross(v_shoulder22shoulder1,v_shoulder12elbow))
    angle_z = np.arctan2(sinang_z,cosang_z)

    #Bind the Angles in the Array
    F.data = [angle_x,angle_z]
    print F.data

    #Publish the MultiArray
    pub.publish(F)

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