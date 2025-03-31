import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import re


import random
import numpy as np

import time

class KinovaMonitorNode(Node):
    def __init__(self, queue):
        super().__init__('kinova_monitor_node')
        
        # Subscriber to get current joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscriber to get end-effector pose
        self.ee_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/end_effector_pose',  # Change this if needed
            self.ee_pose_callback,
            10
        )
        self.joint_positions = [0, 0, 0, 0, 0, 0, 0]
        self.queue = queue
    
    def joint_state_callback(self, msg):
        """
        Callback to print current joint positions
        """

        joint_positions_dict = {}

        for name, position in zip(msg.name, msg.position):
            if "joint_" in name:
                joint_positions_dict[name] = position


        sorted_joint_names = sorted(joint_positions_dict.keys(), key=lambda name: int(re.search(r'(\d+)', name).group(1)))
        
        # Sorted positions based on the sorted joint names
        sorted_positions = [joint_positions_dict[name] for name in sorted_joint_names]
        self.queue.put(sorted_positions)
        # print("sorted positions", sorted_positions)
        # print("Queue size after put:", self.queue.qsize())

        
        
        # Print the sorted joint names and their corresponding positions
        for joint_name, position in zip(sorted_joint_names, sorted_positions):
            #self.get_logger().info(f'{joint_name}: {position}')
            #self.get_logger().info(f'{sorted_positions}')
            pass


        #self.get_logger().info(f'Current Joint Positions: {msg.position}')

    
    def ee_pose_callback(self, msg):
        """
        Callback to print current end-effector position
        """
        
        self.get_logger().info(f'Current End-Effector Position: {msg.pose.position}')


def main(args=None):
    rclpy.init(args=args)
    node = KinovaMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
