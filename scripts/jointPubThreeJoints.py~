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
window_size=1
active_joints=3
prev_angles=np.zeros((window_size,active_joints))
stable_angles=np.zeros((window_size,active_joints))

class JointMapper():
    def __init__(self, left, grip_left, lj):
        self.left = left
        self.grip_left = grip_left
        self.lj = lj
    def set_j(self,limb, joint_names, angles,speed):
        #current_position = limb.joint_angle(joint_names)
        joint_command = {joint_names[0]: angles[0],joint_names[1]: angles[1],joint_names[2]: angles[2]}
        # speed_command = {joint_name: speed}
        # limb.set_joint_velocities(speed_command)
        limb.set_joint_positions(joint_command)

    def callback(self,data):

        # Get joint Markers from Bag file
        shoulder_left = data.markers[2].translation
        torso = data.markers[0].translation
        shoulder_right = data.markers[1].translation
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

        b_shoulderPitch = 1.048859 - 0.01858033*(shoulderPitch_angle-25) #beech me minus he
        b_shoulderYaw = -1.28969 + 0.023713529*(shoulderYaw_angle-83)
        b_elbowPitch = 2.606616 - 0.02124562*(elbowPitch_angle-55)

        # angle_list.append(b_shoulderPitch)
	
        del_shoulder_pitch=abs(b_shoulderPitch-prev_angles[0][1])
        del_shoulder_yaw=abs(b_shoulderYaw-prev_angles[0][0])
        del_elbow_pitch=abs(b_elbowPitch-prev_angles[0][2])
        global prev_angles
        global window_size
        speed=2
        f = Float32MultiArray()
	f.data = [0,0,0]
   
        # shoulder pitch
        if (del_shoulder_pitch<0.9):
			# print b_shoulderYaw+np.sum(prev_angles[:][1]),b_shoulderPitch+np.sum(prev_angles[:][0]),b_elbowPitch+np.sum(prev_angles[:][2])
		#self.set_j(self.left,self.lj[1],(b_shoulderPitch),speed);
		setShoulderPitch = b_shoulderPitch
		f.data[0] = b_shoulderPitch
		prev_angles[0][1]=b_shoulderPitch
		stable_angles[0][1] = b_shoulderPitch
	else:
		#self.set_j(self.left,self.lj[1],(stable_angles[0][1]),speed)
		setShoulderPitch = stable_angles[0][1]
		prev_angles[0][1] = b_shoulderPitch
		f.data[0] = stable_angles[0][1]
        # shoulder yaw	
	if (del_shoulder_yaw<0.7):
		#self.set_j(self.left,self.lj[0],(b_shoulderYaw),speed)
		setShoulderYaw = b_shoulderYaw
		f.data[1] = b_shoulderYaw		
		prev_angles[0][0]=b_shoulderYaw	
		stable_angles[0][0] = b_shoulderYaw
	else:
		#self.set_j(self.left,self.lj[0],(prev_angles[0][0]),speed);	
		setShoulderYaw = stable_angles[0][0]
		prev_angles[0][0] = b_shoulderPitch	
		f.data[1] = stable_angles[0][1]
        # elbow pitch
	if (del_elbow_pitch<0.7):
		#self.set_j(self.left,self.lj[3],(b_elbowPitch),speed);
		setElbowPitch = b_elbowPitch
		prev_angles[0][2]=b_elbowPitch
		f.data[2] = b_elbowPitch
		stable_angles[0][2] = b_elbowPitch
	else:
		#self.set_j(self.left,self.lj[3],(prev_angles[0][2]),speed);
		setElbowPitch = stable_angles[0][2]
		prev_angles[0][2] = b_elbowPitch
		f.data[2] = stable_angles[0][2]
	
	self.set_j(self.left,[self.lj[0],self.lj[1],self.lj[3]],[setShoulderYaw, setShoulderPitch, setElbowPitch],speed);
	pub.publish(f)
       # for i in range(1,np.shape(prev_angles)[0]):
	#        prev_angles[i][:]=prev_angles[i-1][:]

	

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

