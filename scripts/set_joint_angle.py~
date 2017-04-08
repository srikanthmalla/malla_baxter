#!/usr/bin/env python
import rospy
#checking by changing joint angles
# baxter_interface - Baxter Python API
import baxter_interface
from baxter_interface import CHECK_VERSION

# initialize our ROS node, registering it with the Master
rospy.init_node('Home_Arms')
rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
rate = rospy.Rate(10) # 10hz
# def clean_shutdown():
#     print("\nExiting example...")
#     if not init_state:
#         print("Disabling robot...")
#         rs.disable()
# rospy.on_shutdown(clean_shutdown)
print("Enabling robot... ")
rs.enable()

# create instances of baxter_interface's Limb class
limb_right = baxter_interface.Limb('right')
limb_left = baxter_interface.Limb('left')
pi=3.147
# store the home position of the arms
home_right = {'right_s0': -1.109, 'right_s1': -1.4457, 'right_w0': 0.3259, 'right_w1': 1.973, 'right_w2': -3.039, 'right_e0': -0.195, 'right_e1': 0.78}
home_left = {'left_s0': 0.25, 'left_s1': 0.5, 'left_w0': -0.23, 'left_w1': -0.15, 'left_w2': 0, 'left_e0': 0.42, 'left_e1': -0.21}
print "before:",limb_left.endpoint_pose()

# move both arms to home position
# limb_right.move_to_joint_positions(home_right)
limb_left.move_to_joint_positions(home_left)
# print limb_right.endpoint_pose()
print "after",limb_left.endpoint_pose()
while not rospy.is_shutdown():
	limb_left.move_to_joint_positions(home_left)
	# print limb_right.endpoint_pose()
	print "after",limb_left.endpoint_pose()
	rate.sleep();


# quit()