#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry TF Publisher started')
    
    def odom_callback(self, msg):
        try:
            # Create transform message
            t = TransformStamped()
            
            # Copy header
            t.header.stamp = msg.header.stamp
            t.header.frame_id = msg.header.frame_id  # 'odom'
            t.child_frame_id = msg.child_frame_id     # 'base_link'
            
            # Ensure child_frame_id is not empty (sometimes bridge sends empty)
            if not t.child_frame_id:
                t.child_frame_id = 'base_link'

            # Copy pose
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            
            t.transform.rotation = msg.pose.pose.orientation
            
            # Broadcast transform
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
