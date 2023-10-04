#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class TF2LookupNode(Node):

    def __init__(self):
        super().__init__('tf2_lookup_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.source_frame = "tool0"
        self.target_frame = "base_link"
        self.create_timer(0.40, self.lookup_transform)  # Call every 1 second

    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time())
            self.get_logger().info('Transform (translation): x={:.2f}, y={:.2f}, z={:.2f}'.format(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ))
            self.get_logger().info('Transform (rotation): x={:.2f}, y={:.2f}, z={:.2f}, w={:.2f}'.format(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ))
        except Exception as e:
            self.get_logger().warn('TF Lookup failed: {}'.format(e))

def main(args=None):
    rclpy.init(args=args)
    node = TF2LookupNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
