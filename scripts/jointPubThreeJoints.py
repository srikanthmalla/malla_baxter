import rospy
import numpy as np
from vicon_bridge.msg import Markers
from std_msgs.msg import Float32MultiArray
import math
import matplotlib.pyplot as plt

pub = rospy.Publisher('/jointangles', Float32MultiArray, queue_size=10)
angle_list = []


def callback(data):
    
    # Get joint Markers from Bag file
    shoulder_left = data.markers[0].translation
    torso = data.markers[1].translation
    shoulder_right = data.markers[2].translation
    elbow_left = data.markers[4].translation
    wrist_left = data.markers[6].translation

    #Create Vectors

    #For Shoulder Pitch
    shoulderPitch_v0 = np.array([shoulder_left.x,shoulder_left.y,shoulder_left.z]) - np.array([torso.x,torso.y,torso.z])
    shoulderPitch_v1 = np.array([shoulder_left.x, shoulder_left.y, shoulder_left.z]) - np.array([elbow_left.x, elbow_left.y, elbow_left.z])
    
    dot_product = np.dot(shoulderPitch_v0,shoulderPitch_v1)
    mag_v0 = np.linalg.norm(shoulderPitch_v0)
    mag_v1 = np.linalg.norm(shoulderPitch_v1)

    shoulderPitch_angle = (180/math.pi)*np.arccos(1.0*(dot_product)/(mag_v0*mag_v1))
    #angle_list.append(shoulderPitch_angle)

    #For Shoulder Yaw
    shoulderYaw_v0 = np.array([shoulder_left.x,shoulder_left.y,shoulder_left.z]) - np.array([shoulder_right.x,shoulder_right.y,shoulder_right.z])
    shoulderYaw_v1 = np.array([shoulder_left.x, shoulder_left.y, shoulder_left.z]) - np.array([elbow_left.x, elbow_left.y, elbow_left.z])

    dot_product = np.dot(shoulderYaw_v0,shoulderYaw_v1)
    mag_v0 = np.linalg.norm(shoulderYaw_v0)
    mag_v1 = np.linalg.norm(shoulderYaw_v1)

    shoulderYaw_angle = (180/math.pi)*np.arccos(1.0*(dot_product)/(mag_v0*mag_v1))
    #angle_list.append(shoulderYaw_angle)

    #For Elbow Pitch
    elbowPitch_v0 = np.array([elbow_left.x,elbow_left.y,elbow_left.z]) - np.array([shoulder_left.x,shoulder_left.y,shoulder_left.z])
    elbowPitch_v1 = np.array([elbow_left.x, elbow_left.y, elbow_left.z]) - np.array([wrist_left.x, wrist_left.y, wrist_left.z])

    dot_product = np.dot(elbowPitch_v0,elbowPitch_v1)
    mag_v0 = np.linalg.norm(elbowPitch_v0)
    mag_v1 = np.linalg.norm(elbowPitch_v1)

    elbowPitch_angle = (180/math.pi)*np.arccos(1.0*(dot_product)/(mag_v0*mag_v1))
    #angle_list.append(elbowPitch_angle)

    b_shoulderPitch = -1.180781 + 0.01858033*(shoulderPitch_angle-25)
    b_shoulderYaw = -1.28969 + 0.023713529*(shoulderYaw_angle-83)
    b_elbowPitch = -0.049087 + 0.02124562*(elbowPitch_angle-55)

    angle_list.append(b_shoulderPitch)

    f = Float32MultiArray()
    f.data = [(b_shoulderPitch, b_shoulderYaw,b_elbowPitch)]
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
    plt.plot(angle_list)
    plt.ylabel('angle')
    plt.show()

