#!/usr/bin/env python
#author srikanth malla

import rospy
from std_msgs.msg import Float32MultiArray
import baxter_interface
from baxter_interface import CHECK_VERSION


def set_j(limb, joint_name, angle,speed):
    current_position = limb.joint_angle(joint_name)
    joint_command = {joint_name: angle}
    speed_command = {joint_name: speed}
    limb.set_joint_positions(speed_command)
    limb.set_joint_positions(joint_command)
def angles_callback(data):
    set_angles=data.data;
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()
    print(lj)
    #['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    # s0 is shoulder twist (-1.7016, 1.7016), s1 is shoulder bend(-2.147, +1.047)
    # e0 is elbow twist(-3.0541, 3.0541), e1 is elbow bend(-0.05,+2.618)
    # w0 is writst twist(-3.059, 3.059), w1 is wrist bend(-1.5707,+2.094), w2 is writst twist 2(-3.059,3.059)
    index=0;
    get_angles=left.joint_angles()
    print("angles to be moved")
    print(set_angles[index])
    print("current angles")
    print(get_angles['left_s0'])
    ang=set_angles[index]*(3.4/2.4)-1.7016
    set_j(left,lj[index],ang,0.3);
    head=baxter_interface.Head();
    head.set_pan(0, speed=0.5)
    print("Done.")



def main():
    print("Initializing node... ")
    rospy.sleep(10);
    rospy.init_node("set_joint_angles") #init rosnode
    rospy.Subscriber("/jointangles",Float32MultiArray, angles_callback)
    # pub = rospy.Publisher('robot_state',Float32MultiArray, queue_size=10)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rate = rospy.Rate(10) # 10hz
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    print("Enabling robot... ")
    rs.enable()
    rate = rospy.Rate(10) # 10hz
    rospy.spin()
    # while not rospy.is_shutdown():
    #     rospy.spin()
        # rate.sleep()



if __name__ == '__main__':
    main()
