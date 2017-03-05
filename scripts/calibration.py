#!/usr/bin/env python
import rospy
import numpy as np
from vicon_bridge.msg import Markers
from std_msgs.msg import Float32MultiArray
import tf

data_collected=[]
#for right hand manipulation
def callback(data):
    
    m_P_torso = m_P_right = m_P_left = None
    markers = {}
    for marker in data.markers:
        markers[marker.marker_name] = np.array([[marker.translation.x],[marker.translation.y],[marker.translation.z]])
    
    # if occluded return
    if "torso" not in markers or "right" not in markers or "left" not in markers or "ee" not in markers: return 



    # Find normal to torso of human
    torso_norm = np.cross(markers["left"] - markers["torso"], markers["right"] - markers["torso"], axis = 0)
    
    # Project torso normal to xy plane of the world frame
    torso_norm[2][0] = 0
    
    # Take X axis as projected torso normal. Find X, Y ,Z axis of the torso frame wrt map
    m_X_tor = torso_norm/np.linalg.norm(torso_norm)
    m_Z_tor = np.array([[0],[0],[1]])
    m_Y_tor = np.cross(m_Z_tor, m_X_tor, axis = 0)
    
    # Find origin of the marker frame with respect to map
    m_P_tor = markers["torso"] # this is P
    
    # Rotation matrix from tor to map. 
    tor_R_m = np.matrix([np.reshape(m_X_tor, 3), np.reshape(m_Y_tor, 3), np.reshape(m_Z_tor, 3)]) # this is R^T if R = m_R_tor
    
    m_P_ee = markers["ee"]
    # tor_P_ee = tor_T_m * m_P_ee
    tor_P_ee = tor_R_m * m_P_ee - tor_R_m * m_P_tor




def listener():
    rospy.init_node('calibration', anonymous=True)
    rospy.Subscriber("/vicon/markers", Markers, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()