#!/usr/bin/env python
import rospy
import numpy as np
from vicon_bridge.msg import Markers
from std_msgs.msg import Float32MultiArray
import math
import matplotlib.pyplot as plt
import baxter_interface
from baxter_interface import CHECK_VERSION

pub = rospy.Publisher('/jointangles', Float32MultiArray, queue_size=10)
angle_list = []
# global prev_angles
window_size=100
active_joints=3
prev_angles=np.zeros((window_size,active_joints))

class JointMapper():
    def __init__(self, left, grip_left, lj):
        self.left = left
        self.grip_left = grip_left
        self.lj = lj
    def set_j(self,limb, joint_name, angle,speed):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: angle}
        # speed_command = {joint_name: speed}
        # limb.set_joint_velocities(speed_command)
        limb.set_joint_positions(joint_command)

    def callback(self,data):

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

        b_shoulderPitch = 1.048859 - 0.01858033*(shoulderPitch_angle-25)
        b_shoulderYaw = -1.28969 + 0.023713529*(shoulderYaw_angle-83)
        b_elbowPitch = 2.606616 - 0.02124562*(elbowPitch_angle-55)

        # angle_list.append(b_shoulderPitch)

        f = Float32MultiArray()
        f.data = [b_shoulderPitch, b_shoulderYaw,b_elbowPitch]
        pub.publish(f)
        del_shoulder_pitch=abs(b_shoulderPitch-prev_angles[0][0])
        del_shoulder_yaw=abs(b_shoulderYaw-prev_angles[0][1])
        del_elbow_pitch=abs(b_elbowPitch-prev_angles[0][2])
        global prev_angles
        global window_size
        speed=2
        # print np.sum(prev_angles,axis=0)
        if (del_shoulder_pitch<100)and(del_shoulder_yaw<100)and(del_elbow_pitch<100):
			# print b_shoulderYaw+np.sum(prev_angles[:][1]),b_shoulderPitch+np.sum(prev_angles[:][0]),b_elbowPitch+np.sum(prev_angles[:][2])
			self.set_j(self.left,self.lj[0],(b_shoulderYaw+np.sum(prev_angles,axis=0)[1])/(window_size+1),speed)
			self.set_j(self.left,self.lj[1],(b_shoulderPitch+np.sum(prev_angles,axis=0)[0])/(window_size+1),speed);
			self.set_j(self.left,self.lj[3],(b_elbowPitch+np.sum(prev_angles,axis=0)[2])/(window_size+1),speed);
        for i in range(1,np.shape(prev_angles)[0]):
	        prev_angles[i][:]=prev_angles[i-1][:]

		prev_angles[0][:]=[b_shoulderPitch,b_shoulderYaw,b_elbowPitch]
		# rate = rospy.Rate(10) 
		# rate.sleep()
        # if ((b_shoulderYaw>-1.2896943474146616)|(b_shoulderYaw<0.7259564078667721))and(b_shoulderPitch>-1.1807817114747972)|(b_shoulderPitch<1.0488593637166517)and(b_elbowPitch>-0.04908738521233324)|(b_elbowPitch<2.6066168538142893):
        # self.set_j(self.left,self.lj[0],b_shoulderYaw,2)
        # self.set_j(self.left,self.lj[1],b_shoulderPitch,2);
        # self.set_j(self.left,self.lj[3],b_elbowPitch,2);


        #['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        # s0 is shoulder twist (-1.7016, 1.7016), s1 is shoulder bend(-2.147, +1.047)
        # e0 is elbow twist(-3.0541, 3.0541), e1 is elbow bend(-0.05,+2.618)
        # w0 is writst twist(-3.059, 3.059), w1 is wrist bend(-1.5707,+2.094), w2 is writst twist 2(-3.059,3.059)
 		
 		# wanted limits
		# shoulder_yaw (s0) range:[-1.2896943474146616,0.7259564078667721]
		# shoulder_pitch (s1) range:[-1.1807817114747972,1.0488593637166517]
		# elbow-roll (e0) range:[-3.05 3.05]
		# elbow_pitch (e1) range:[-0.04908738521233324,2.6066168538142893]


 
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rate = rospy.Rate(100) # 10hz
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    print("Enabling robot... ")
    rs.enable()
    left= baxter_interface.Limb('left')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    lj = left.joint_names()
    jm = JointMapper(left,grip_left,lj)
    rospy.Subscriber("/vicon/markers", Markers, jm.callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    jm.listener()

