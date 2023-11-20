import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from tf2_ros import TransformListener, Buffer
from arc_interfaces.srv import ArcTargetPose
from arc_interfaces.msg import ArcHPComm
from geometry_msgs.msg import TransformStamped

import threading

cuurent_pose = None

class SimpleSubNode(Node):

    def __init__(self):
        super().__init__('simple_sub_node')
        self.sub = self.create_subscription(TransformStamped, '/tf2_lookup_node/transform', self.transform_callback, 1)
        self.curr_pose = None

    def transform_callback(self, msg: TransformStamped):
        self.get_logger().info("Received transform message")
        self.curr_pose = msg.transform.translation

class SuccessSubNode(Node):

    def __init__(self):
        super().__init__('simple_sub_node')
        self.sub = self.create_subscription(ArcHPComm, '/arc_planning_interface/hp_comm', self.callback, 1)
        self.is_finished = None

    def callback(self, msg: ArcHPComm):
        self.get_logger().info("Received ArcHPComm message. status: " + str(msg.status))
        if msg.status == 1:
            self.is_finished = True

    def stop(self):
        self.sub.destroy()


class LookUpNode(Node):
    
        def __init__(self, x, y, z):
            super().__init__('look_up_node')
            self.x = x
            self.y = y
            self.z = z
            self.source_frame = "tool0"
            self.target_frame = "base_link"
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(0.40, self.lookup_transform)
            self.goal_reached = False
    
        def lookup_transform(self):
            try:
                transform = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time())
                self.get_logger().info('Transform (translation): x={:.2f}, y={:.2f}, z={:.2f}'.format(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ))
                if abs(transform.transform.translation.x - self.x) < 0.01 and abs(transform.transform.translation.y - self.y) < 0.01 and abs(transform.transform.translation.z - self.z) < 0.01:
                    self.get_logger().info("Reached goal")
                    self.goal_reached = True
                    self.timer.cancel()
                    return
            except Exception as e:
                self.get_logger().warn('TF Lookup failed: {}'.format(e))
        
        def stop(self):
            self.timer.cancel()

class ServiceClientNode(Node):

    def __init__(self):
        super().__init__('service_client_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._move_target_client = self.create_client(ArcTargetPose, '/arc_planning_interface/move_to_target_pose')
        self.cancel_client = self.create_client(Trigger, '/arc_planning_interface/cancel_all_goals')
        #self.sub = self.create_subscription(TransformStamped, '/tf2_lookup_node/transform', self.transform_callback, 1)
        #self.curr_pose = None

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

        look_up_node = LookUpNode(x, y, z)
        lookup_thread = threading.Thread(target=lambda: rclpy.spin(look_up_node))
        lookup_thread.start()

        start_time = time.time()
        timeout = 30  # seconds

        while True:
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout reached in move_blocking")
                break

            if look_up_node.goal_reached:
                self.get_logger().info("Goal reached in move_blocking")
                break

            time.sleep(0.1)  # Small delay to prevent high CPU usage

        lookup_thread.join()  # Wait for the thread to finish

    def move_sub(self, sub_node : SuccessSubNode, x, y, z):
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
        start_time = time.time()
        timeout = 30

        while True:
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout reached in move_sub")
                sub_node.stop()
                break
            if sub_node.is_finished == True:
                break
            time.sleep(0.1)

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

    def loop_movement(self, sub_node : SuccessSubNode):
        try:
            while True:
                self.move_sub(sub_node, 0.17, 0.6, 0.4)
                time.sleep(5)
                self.move_sub(sub_node, -0.4, 0.4, 0.4)
                time.sleep(5)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt detected. Cancelling all goals.")
            self.send_cancel_request()

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientNode()
    sub_node = SuccessSubNode()
    sub_thread = threading.Thread(target=lambda: rclpy.spin(sub_node))
    sub_thread.start()
    print("Enter to start loop movement")
    input()
    node.loop_movement(sub_node)
    rclpy.spin(node)
    sub_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
