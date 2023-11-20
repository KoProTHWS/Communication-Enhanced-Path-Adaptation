import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from tf2_ros import TransformListener, Buffer
from arc_interfaces.srv import ArcTargetPose
from arc_interfaces.msg import ArcHPComm
from geometry_msgs.msg import TransformStamped

class ServiceClientNode(Node):

    def __init__(self):
        super().__init__('service_client_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._move_target_client = self.create_client(ArcTargetPose, '/arc_planning_interface/move_to_target_pose')
        self.cancel_client = self.create_client(Trigger, '/arc_planning_interface/cancel_all_goals')
        self.hp_comm_subscriber = self.create_subscription(ArcHPComm, '/arc_planning_interface/hp_comm', self.hp_comm_callback, 1)
        self.goal_status = None
        self.source_frame = "tool0"
        self.target_frame = "base_link"
        self.curr_pose = None
        self.sub = self.create_subscription(TransformStamped, '/tf2_lookup_node/transform', self.transform_callback, 1)

    def hp_comm_callback(self, msg: ArcHPComm):
        self.get_logger().info("Received HP comm message: %s" % msg.message)
        self.goal_status = msg.status

    def send_move_request(self, x, y, z):
        if not self._move_target_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Move target service is not available')
            return

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = 1.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0

        request = ArcTargetPose.Request()
        request.pose = target_pose

        future = self._move_target_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result().success}')
        else:
            self.get_logger().error('Exception while calling move target service: %r' % future.exception())

    def move_blocking(self, x, y, z):
        self.goal_status = 0
        self.send_move_request(x, y, z)

        start_time = time.time()
        timeout = 10  # seconds
        while True:
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout reached in move_blocking")
                break

            if self.curr_pose is None:
                continue
            x_i, y_i, z_i = self.curr_pose.x, self.curr_pose.y, self.curr_pose.z
            self.get_logger().info("Current pose: x=%f, y=%f, z=%f" % (x_i, y_i, z_i))
            if abs(x_i - x) < 0.01 and abs(y_i - y) < 0.01 and abs(z_i - z) < 0.01:
                self.get_logger().info("Reached goal")
                break

    def transform_callback(self, msg: TransformStamped):
        self.get_logger().info("Received transform message")
        self.curr_pose = msg.transform.translation

    def send_cancel_request(self):
        if not self.cancel_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Cancel service is not available')
            return

        request = Trigger.Request()
        future = self.cancel_client.call_async(request)
        
        if future.result() is not None:
            self.get_logger().info(f'Received cancel response: {future.result().success}')
        else:
            self.get_logger().error('Exception while calling cancel service: %r' % future.exception())

    def loop_movement(self):
        try:
            while True:
                self.move_blocking(0.17, 0.6, 0.4)
                time.sleep(2)
                self.move_blocking(-0.4, 0.4, 0.4)
                time.sleep(2)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt detected. Cancelling all goals.")
            self.send_cancel_request()

def main(args=None):
    rclpy.init(args=args)

    node = ServiceClientNode()
    while True:
        try:
            coords = input("Enter x y z (or 'q' to quit, 'c' to cancel, 'l' to loop): ")
            if coords.lower() == 'c':
                node.send_cancel_request()
            elif coords.lower() == 'l':
                node.loop_movement()
            elif coords.lower() == 'q':
                break
            else:
                x, y, z = map(float, coords.split())
                node.send_move_request(x, y, z)
        except ValueError:
            print("Invalid input. Please enter three numbers separated by spaces or 'q' to quit, 'c' to cancel ongoing goals.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
