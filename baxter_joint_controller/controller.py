import threading
import time

import rclpy
from baxter_core_msgs.msg import JointCommand
from baxter_joint_controller.gripper import Gripper
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_ANGLE_TOLERANCE = 0.008726646
HEAD_PAN_ANGLE_TOLERANCE = 0.1396263401

class JointController(Node):

    def __init__(self):
        super().__init__('baxter_joint_controller')
        self.left_publisher = self.create_publisher(JointCommand, '/robot/limb/left/joint_command', 10)
        self.right_publisher = self.create_publisher(JointCommand, '/robot/limb/right/joint_command', 10)
        self._gripper_left = Gripper("left", self, False)
        self._gripper_right = Gripper("right", self, False)
        self.left_thread = threading.Thread(target=self.init_gripper, args=(self._gripper_left,))
        self.left_thread.start()
        self.right_thread = threading.Thread(target=self.init_gripper, args=(self._gripper_right,))
        self.right_thread.start()

        self.subscription = self.create_subscription(
            JointState,
            'joint_states_sim',
            self.listener_callback,
            10)
        self.joint_states = {}
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
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

    def init_gripper(self, gripper:Gripper):
        while not (gripper.state_init and gripper.prop_init):
            time.sleep(.1)
            print("Thread Sleep")
        if gripper.calibrate():
            print(f"Calibrating successfull for {gripper.name}")
        return
        

    def set_joint_positions(self, positions, hand="right", raw=False ):
        """
        Commands the joints of this limb to the specified positions.

        B{IMPORTANT:} 'raw' joint position control mode allows for commanding
        joint positions, without modification, directly to the JCBs
        (Joint Controller Boards). While this results in more unaffected
        motions, 'raw' joint position control mode bypasses the safety system
        modifications (e.g. collision avoidance).
        Please use with caution.
        """

        command_msg = JointCommand()
        if hand not in ["right", "left"]:
            return command_msg
        command_msg.names = self._joint_names[hand]
        command_msg.command = positions
        if raw:
            command_msg.mode = JointCommand.RAW_POSITION_MODE
        else:
            command_msg.mode = JointCommand.POSITION_MODE
        return command_msg

    def listener_callback(self, msg:JointState):
        for name, pose in zip(msg.name, msg.position):
            self.joint_states[name] = pose

    def convert_range(self,value):
        # print(f"Original : {value}, New : {(value * 100 / 0.020833)}")
        return (abs(value) * 100 / 0.020833)


    def timer_callback(self):
        self.right_publisher.publish(self.set_joint_positions([y for x, y in self.joint_states.items() if x in self._joint_names["right"]], hand="right"))
        self.left_publisher.publish(self.set_joint_positions([y for x, y in self.joint_states.items() if x in self._joint_names["left"]], hand="left"))
        if self._gripper_left.calibrated != True:
            try:
                self._gripper_right.command_position(min(self.convert_range(self.joint_states["l_gripper_l_finger_joint"]), self.convert_range(self.joint_states["l_gripper_r_finger_joint"])))
            except KeyError:
                pass
        if self._gripper_right.calibrated != True:
            try:
                self._gripper_right.command_position(min(self.convert_range(self.joint_states["r_gripper_l_finger_joint"]), self.convert_range(self.joint_states["r_gripper_r_finger_joint"])))
            except KeyError:
                pass


def main(args=None):
    rclpy.init(args=args)

    node = JointController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
