#!/usr/bin/env python3

# ROS python API
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.callback_groups import ReentrantCallbackGroup

# Flight modes class
class FCUModes:
    def __init__(self, node):
        self.node = node

    def set_arm(self):
        # Wait for the service to become available
        self.node.get_logger().info('Waiting for arm service...')
        arm_service = self.node.create_client(CommandBool, 'mavros/cmd/arming')

        while not arm_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Arming service not available, waiting...')

        # Create request and send it to arm the drone
        request = CommandBool.Request()
        request.value = True
        arm_service.call_async(request)

    def set_disarm(self):
        self.node.get_logger().info('Waiting for disarm service...')
        arm_service = self.node.create_client(CommandBool, 'mavros/cmd/arming')

        while not arm_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Disarming service not available, waiting...')

        # Create request and send it to disarm the drone
        request = CommandBool.Request()
        request.value = False
        arm_service.call_async(request)

    def set_mode(self, mode: str):
        self.node.get_logger().info(f'Waiting for mode service to set {mode}...')
        mode_service = self.node.create_client(SetMode, 'mavros/set_mode')

        while not mode_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Set mode service not available, waiting for {mode} mode...')

        # Create request and send mode change command
        request = SetMode.Request()
        request.custom_mode = mode
        mode_service.call_async(request)


# Main controller class
class Controller(Node):
    def __init__(self):
        super().__init__('takeoff_offboard_test_node')

        # Drone state and setpoint messages
        self.state = State()
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)  # Only position control
        self.sp.coordinate_frame = 1  # LOCAL_NED

        # Altitude setpoint
        self.ALT_SP = self.declare_parameter('alt_sp', 1.0).get_parameter_value().double_value
        self.sp.position.z = self.ALT_SP

        # Local position of drone
        self.local_pos = Point(0.0, 0.0, 0.0)

        # Subscription for local position
        self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pos_callback, 10)
        # Subscription for state
        self.create_subscription(State, 'mavros/state', self.state_callback, 10)

        # Publisher for position setpoints
        self.sp_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', 10)

    def pos_callback(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    def state_callback(self, msg):
        self.state = msg

    def update_setpoint(self):
        # Update setpoint based on current position and joystick commands (if any)
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y
        self.sp.position.z = self.ALT_SP


def main():
    rclpy.init()

    # Create Node
    node = Controller()

    # Create FCUModes object to control flight modes
    modes = FCUModes(node)

    # Create a rate for loop
    rate = node.create_rate(20.0)  # 20 Hz

    # Make sure the drone is armed
    while not node.state.armed:
        modes.set_arm()
        rate.sleep()

    # Wait for a few seconds to ensure the drone is in position
    for _ in range(10):
        node.update_setpoint()
        node.sp_pub.publish(node.sp)
        rate.sleep()

    # Main loop
    while rclpy.ok():
        node.sp_pub.publish(node.sp)
        rate.sleep()

    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
