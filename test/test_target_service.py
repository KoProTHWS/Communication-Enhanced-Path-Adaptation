import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from arc_interfaces.srv import ArcTargetPose

class ServiceClientNode(Node):

    def __init__(self):
        super().__init__('service_client_node')
        self._move_target_client = self.create_client(ArcTargetPose, '/arc_planning_interface/move_to_target_pose')
        self.cancel_client = self.create_client(Trigger, '/arc_planning_interface/cancel_all_goals')

    def send_move_request(self, x, y, z):
        if not self._move_target_client.wait_for_service(timeout_sec=5.0):
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

        future = self._move_target_client.call_async(request)
        #rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result().response}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

    def send_cancel_request(self):
        if not self.cancel_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service is not available')
            return

        request = Trigger.Request()
        future = self.cancel_client.call_async(request)
        #rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result().success}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)

    node = ServiceClientNode()
    while True:
        try:
            coords = input("Enter x y z (or 'q' to quit): ")
            if 'c' in coords:
                node.send_cancel_request()
                continue
            elif 'q' in coords:
                break
            x, y, z = map(float, coords.split())
            node.send_move_request(x, y, z)
        except ValueError:
            print("Invalid input. Please enter three numbers separated by spaces or 'q' to quit. c to cancel ongoing goals.")
            continue
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
