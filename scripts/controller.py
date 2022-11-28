#!/usr/bin/python3
import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import JointState



JOINT_ANGLE_TOLERANCE = 0.008726646
HEAD_PAN_ANGLE_TOLERANCE = 0.1396263401

class JointController():

    def __init__(self):
        # self.left_publisher = rospy.Publisher('/robot/limb/left/joint_command',JointCommand, 10)
        # self.right_publisher = rospy.Publisher('/robot/limb/right/joint_command',JointCommand, 10)
        
        self.left = baxter_interface.Limb('left')
        self.right = baxter_interface.Limb('right')
        self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.grip_right = baxter_interface.Gripper('right', CHECK_VERSION)

        self.subscription = rospy.Subscriber(
            'joint_states_sim',
            JointState,
            self.listener_callback
        )
        self.joint_states = {}

        self.i = 0
        self._joint_names = {
            'left': ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                     'left_w0', 'left_w1', 'left_w2'],
            'right': ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                      'right_w0', 'right_w1', 'right_w2'],
            # 'left_gripper' : ["l_gripper_l_finger_joint",
            #                 "l_gripper_r_finger_joint",],
            # 'right_gripper' : ["r_gripper_l_finger_joint",
            #                 "r_gripper_r_finger_joint",]
            }

    def listener_callback(self, msg:JointState):
        for name, pose in zip(msg.name, msg.position):
            self.joint_states[name] = pose

    def convert_range(self,value):
        # print(f"Original : {value}, New : {(value * 100 / 0.020833)}")
        return (value * 100 / 0.020833)

    def timer_callback(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            self.right.set_joint_positions({x:y for x, y in self.joint_states.items() if x in self._joint_names["right"]})
            self.left.set_joint_positions({x:y for x, y in self.joint_states.items() if x in self._joint_names["left"]})
            try:
                self.grip_left.command_position(self.convert_range(self.joint_states["l_gripper_l_finger_joint"]))
            except KeyError:
                pass
            try:
                self.grip_right.command_position(self.convert_range(self.joint_states["r_gripper_l_finger_joint"]))
            except KeyError:
                pass
            rate.sleep()

def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_controller")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    print("Enabling robot... ")
    rs.enable()
    print("Done.")
    controller = JointController()
    controller.timer_callback()



if __name__ == '__main__':
    main()