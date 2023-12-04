#!usr/bin/python3

import rclpy
from turtlesim.msg import Pose
from rclpy.node import Node

class PoseSubcriber(Node):

    def __init__(self):
        super().__init__("pose_subscriber", parameter_overrides=[])
        self.pose_subscriber = self.create_subscription(msg_type=Pose, topic="/turtle1/pose", callback=self.pose_callback, qos_profile=10)

    def pose_callback(self, msg: Pose):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubcriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()