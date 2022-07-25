import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)
from json import (
    JSONDecoder,
    JSONEncoder,
)

from baxter_core_msgs.msg import (
    EndEffectorCommand,
    EndEffectorProperties,
    EndEffectorState,
)

JOINT_ANGLE_TOLERANCE = 0.008726646
HEAD_PAN_ANGLE_TOLERANCE = 0.1396263401

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.left_publisher = self.create_publisher(JointCommand, '/robot/limb/left/joint_command', 10)
        self.right_publisher = self.create_publisher(JointCommand, '/robot/limb/right/joint_command', 10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states_sim',
            self.listener_callback,
            10)
        self.joint_states = {}
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self._joint_names = {
            'left': ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                     'left_w0', 'left_w1', 'left_w2'],
            'right': ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                      'right_w0', 'right_w1', 'right_w2']
            }
        

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
        print(f"{hand}:{positions}")
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

    def timer_callback(self):
        self.right_publisher.publish(self.set_joint_positions([y for x, y in self.joint_states.items() if x in self._joint_names["right"]], hand="right"))
        self.left_publisher.publish(self.set_joint_positions([y for x, y in self.joint_states.items() if x in self._joint_names["left"]], hand="left"))



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
