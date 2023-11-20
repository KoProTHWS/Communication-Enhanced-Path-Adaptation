import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from tf2_ros import TransformListener, Buffer
from arc_interfaces.srv import ArcTargetPose
from arc_interfaces.msg import ArcHPComm
from geometry_msgs.msg import TransformStamped

class SimpleSubNode(Node):

    def __init__(self):
        super().__init__('simple_sub_node')
        self.sub = self.create_subscription(TransformStamped, '/tf2_lookup_node/transform', self.transform_callback, 1)
        self.curr_pose = None

    def transform_callback(self, msg: TransformStamped):
        self.get_logger().info("Received transform message")
        self.curr_pose = msg.transform.translation

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
