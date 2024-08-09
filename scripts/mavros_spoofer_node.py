#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovariance

class MavrosSpooferNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.fortress_odom = self.create_subscription(
            PoseArray,
            '/model/wanderer_II/odometry',
            self.fortress_odom_callback,
            10
        )

    def fortress_odom_callback(self, msg):
        pose = msg.pose.pose
        posX = pose.position.x
        posY = pose.position.y
        posZ = pose.position.z
        self.get_logger().info('X: %s ' % posX)
        self.get_logger().info('Y: %s ' % posY)
        self.get_logger().info('Z: %s ' % posZ)

def main(args=None):
    rclpy.init(args=args)

    mavros_spoofer = MavrosSpooferNode()

    rclpy.spin(mavros_spoofer)

    # Destroy the node explicitly
    mavros_spoofer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()