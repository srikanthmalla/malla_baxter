import rospy
import numpy as np
from vicon_bridge.msg import Markers
from std_msgs.msg import Float32MultiArray
import math
import matplotlib.pyplot as plt

pub = rospy.Publisher('/jointangles', Float32MultiArray, queue_size=10)
angle_list = []


def callback(data):
    
    shoulder_left = data.markers[0].translation;
    torso = data.markers[1].translation
    elbow_left = data.markers[4].translation
    v0 = np.array([shoulder_left.x,shoulder_left.y,shoulder_left.z]) - np.array([torso.x,torso.y,torso.z])
    v1 = np.array([shoulder_left.x, shoulder_left.y, shoulder_left.z]) - np.array([elbow_left.x, elbow_left.y, elbow_left.z])
    
    dot_product = np.dot(v0,v1)
    mag_v0 = np.linalg.norm(v0)
    mag_v1 = np.linalg.norm(v1)
    #angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
    #print angle
    # rospy.loginfo(rospy.get_caller_id() + "I heard %f", angle)
    angle = (180/math.pi)*np.arccos(1.0*(dot_product)/(mag_v0*mag_v1))
    angle_list.append(angle)

    # sinang = np.linalg.norm(np.cross(v0,v1))
    # angle = np.arctan2(sinang,dot_product)
    # angle_list.append(angle)

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
    plt.plot(angle_list)
    plt.ylabel('angle')
    plt.show()
