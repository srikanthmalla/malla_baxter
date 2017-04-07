#!/usr/bin/env python

# this is for controls project data collection

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import matplotlib.pyplot as plt


if __name__ == '__main__':
    rospy.init_node('data_collection', anonymous=True)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rate = rospy.Rate(100) # 10hz
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    # rospy.on_shutdown(clean_shutdown)
    print("Enabling robot... ")
    rs.enable()
    left= baxter_interface.Limb('left')
    lj = left.joint_names()
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    data = []
    count = 0
    t = []
    while not rospy.is_shutdown():
    	print "setting torque of 10"
    	torque_command={lj[1]:-10}
    	left.set_joint_torques(torque_command)
    	jointVal = left.joint_angle(lj[1])
    	t.append(count)
    	data.append(jointVal)
    	count = count + 1
    	print "Joint Angle: ", jointVal


    plt.plot(t, data)
    plt.ylabel('Angle in radians')
    plt.xlabel('Time')
    plt.show()
