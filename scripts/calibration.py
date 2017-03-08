#!/usr/bin/env python
import rospy
import argparse
import sys
import numpy as np
from vicon_bridge.msg import Markers
from std_msgs.msg import Float32MultiArray
from collections import deque
import tf
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

from std_msgs.msg import (
    UInt16,
)
    # <include file="$(find vicon_bridge)/launch/vicon.launch">
    #     <arg name="hostport" value="130.215.206.248" />
    # </include> 
data_collected=[]
#for right hand manipulation

class TeleOp:

    def __init__(self):
        rospy.init_node('calibration', anonymous=True)
        rs=baxter_interface.RobotEnable(CHECK_VERSION)
        init_state=rs.state().enabled
        # print("Enabling the robot")
        rs.enable()
        self._amp=4.0
        self.limb_right=baxter_interface.Limb('right')
        self.right_kinematics=baxter_kinematics('right')
        self.p_markers = None
        self.p_time = None
        self.ee_poses = deque()
        self.max_marker_velocity = rospy.get_param('~max_marker_velocity')
        self.max_window_size = rospy.get_param('~max_window_size')
        self.set_neutral()
        rospy.sleep(5)
        self.vicon_sub = rospy.Subscriber("/vicon/markers", Markers, self.callback)
        self.K = 0 # Initialize from param server

    def set_neutral(self):
        """
       Sets both arms back into a neutral pose.
       """
        # print("Moving to neutral pose...")
        self.limb_right.move_to_neutral()
    def _reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self.limb_right.exit_control_mode()
            rate.sleep()
        return True
    def clean_shutdown(self):
        # print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            # print("Disabling robot...")
            self._rs.disable()
        return True

    def send_joint_velocities(self, joint_velocities):
            cmd = {}
            for idx, name in enumerate(self.limb_right.joint_names()):
                # print idx, name
                cmd[name] =  joint_velocities[idx]* self._amp
            self.limb_right.set_joint_velocities(cmd)   

    def get_pos_control(des_pos, des_jv):
    	x = np.append(np.array([self.limb_right.joint_angles()[i] for i in self.limb_right.joint_names()]), np.array([self.limb_right.joint_velocity()[i] for i in self.limb_right.joint_names()]))
    	x_des = np.append(des_pos, des_jv)
    	e = x - x_des
    	return -self.K*e



    def callback(self, data):

        current_time = rospy.Time.now()
        markers = {}

        for marker in data.markers:
            markers[marker.marker_name] = np.array([[marker.translation.x],[marker.translation.y],[marker.translation.z]])
            if self.p_markers and self.p_time and marker.marker_name in ['ee', 'torso', 'left', 'right']:
                marker_speed = np.abs(markers[marker.marker_name] - self.p_markers[marker.marker_name]) / (current_time - self.p_time).to_sec()
                if any(marker_speed > self.max_marker_velocity):
                    self.send_joint_velocities([0,0,0,0,0,0,0])
                    return
        
        # if occluded return
        if "torso" not in markers or "right" not in markers or "left" not in markers or "ee" not in markers: 
            self.send_joint_velocities([0,0,0,0,0,0,0])
            return 

        # Remember previous poses to filter out noise 
        self.p_markers = markers
        self.p_time = current_time

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

        self.ee_poses.append([tor_P_ee, current_time])

        if len(self.ee_poses) > self.max_window_size: 
            old_pose_time = self.ee_poses.popleft()
            old_pose = old_pose_time[0]
            old_time = old_pose_time[1]

            avg_ee_velocity = ((tor_P_ee - old_pose) / (current_time - old_time).to_sec())/1000.0 #scale down to meters/sec
            joint_angle_dict = self.limb_right.joint_angles()
            jacob=self.right_kinematics.jacobian(joint_angle_dict)[0:3][:]

            joint_velocities=np.linalg.pinv(jacob)*avg_ee_velocity
    
            get_pos_control(self.right_kinematics.inverse_kinematics(position = list(tor_P_ee), seed = [joint_angle_dict[i] for i in self.limb_right.joint_names]), joint_velocities)
            limb_right_names = self.limb_right.joint_names()
            self.send_joint_velocities(joint_velocities)
        	            

    

if __name__ == '__main__':
    teleop_node = TeleOp();
    rospy.spin()