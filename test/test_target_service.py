import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from arc_interfaces.srv import ArcTargetPose

class ServiceClientNode(Node):

    def __init__(self):
        super().__init__('service_client_node')
        self.client = self.create_client(ArcTargetPose, 'move_to_target_pose')

    def send_request(self, x, y, z):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service is not available')
            return

        taget_pose = Pose()
        taget_pose.position.x = x
        taget_pose.position.y = y
        taget_pose.position.z = z

        taget_pose.orientation.x = 1.0
        taget_pose.orientation.y = 0.0
        taget_pose.orientation.z = 0.0
        taget_pose.orientation.w = 0.0

        request = ArcTargetPose.Request()
        request.pose = taget_pose

        future = self.client.call_async(request)
        #rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result().response}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)

    node = ServiceClientNode()
    while True:
        x = float(input("Enter x (or 'q' to quit): "))
        if x == 'q':
            break
        y = float(input("Enter y (or 'q' to quit): "))
        if y == 'q':
            break
        z = float(input("Enter z (or 'q' to quit): "))
        if z == 'q':
            break
        node.send_request(x, y, z)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
