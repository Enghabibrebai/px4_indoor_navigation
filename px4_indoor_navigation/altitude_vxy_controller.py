#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Empty
import tf2_ros
import tf2_geometry_msgs
from math import sin, cos, pi
from .PID import PID  # Correct import of PID class
import time


class FcuModes:
    def __init__(self, node):
        self.node = node

    def set_arm(self):
        try:
            arm_service = self.node.create_client(CommandBool, 'mavros/cmd/arming')
            arm_service.wait_for_service()
            request = CommandBool.Request()
            request.value = True
            arm_service.call(request)
        except Exception as e:
            self.node.get_logger().error(f"Service arming call failed: {e}")

    def set_disarm(self):
        try:
            arm_service = self.node.create_client(CommandBool, 'mavros/cmd/arming')
            arm_service.wait_for_service()
            request = CommandBool.Request()
            request.value = False
            arm_service.call(request)
        except Exception as e:
            self.node.get_logger().error(f"Service disarming call failed: {e}")

    def set_stabilized_mode(self):
        try:
            flight_mode_service = self.node.create_client(SetMode, 'mavros/set_mode')
            flight_mode_service.wait_for_service()
            request = SetMode.Request()
            request.custom_mode = 'STABILIZED'
            flight_mode_service.call(request)
        except Exception as e:
            self.node.get_logger().error(f"Service set_mode call failed: {e}")

    def set_offboard_mode(self):
        try:
            flight_mode_service = self.node.create_client(SetMode, 'mavros/set_mode')
            flight_mode_service.wait_for_service()
            request = SetMode.Request()
            request.custom_mode = 'OFFBOARD'
            flight_mode_service.call(request)
        except Exception as e:
            self.node.get_logger().error(f"Service set_mode call failed: {e}")

    def set_auto_land_mode(self):
        try:
            flight_mode_service = self.node.create_client(SetMode, 'mavros/set_mode')
            flight_mode_service.wait_for_service()
            request = SetMode.Request()
            request.custom_mode = 'AUTO.LAND'
            flight_mode_service.call(request)
        except Exception as e:
            self.node.get_logger().error(f"Service set_mode call failed: {e}")


class Controller(Node):
    def __init__(self):
        super().__init__('altitude_vxy_controller')

        self.mode = FcuModes(self)
        self.state = " "
        self.is_armed = False
        self.landed_state = 1  # on ground
        self.sp = PositionTarget()
        self.sp.type_mask = int('101111000111', 2)
        self.sp.coordinate_frame = 1

        # Parameters
        self.declare_parameter("alt_sp", 1.0)
        self.alt_sp = self.get_parameter("alt_sp").value

        self.declare_parameter("MAX_VUP", 2.0)
        self.MAX_VUP = self.get_parameter("MAX_VUP").value

        self.declare_parameter("MAX_VDOWN", 0.5)
        self.MAX_VDOWN = self.get_parameter("MAX_VDOWN").value

        self.declare_parameter("TRANSFORM_VEL", True)
        self.TRANSFORM_VEL = self.get_parameter("TRANSFORM_VEL").value

        self.declare_parameter("alt_Kp", 1.0)
        self.Kp = self.get_parameter("alt_Kp").value

        self.declare_parameter("alt_Ki", 0.1)
        self.Ki = self.get_parameter("alt_Ki").value

        self.declare_parameter("alt_Kd", 0.01)
        self.Kd = self.get_parameter("alt_Kd").value

        self.declare_parameter("Ts", 0.1)
        self.Ts = self.get_parameter("Ts").value

        self.pid = PID(self.Kp, self.Ki, self.Kd)  # Correct instantiation
        self.pid.setSampleTime(self.Ts)

        # Subscribers
        self.create_subscription(PoseStamped, "mavros/local_position/pose", self.pos_cb, 10)
        self.create_subscription(State, "mavros/state", self.state_cb, 10)
        self.create_subscription(ExtendedState, "mavros/extended_state", self.landing_state_cb, 10)
        self.create_subscription(Empty, "arm", self.arm_cb, 10)
        self.create_subscription(Empty, "disarm", self.disarm_cb, 10)

        # Setpoint Publisher
        self.sp_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', 10)

        # Initialize local position as a Point object with keyword arguments
        self.local_pos = Point(x=0.0, y=0.0, z=0.0)
        self.yaw = 0.0

        # Movement parameters
        self.movement_sequence = [
            (3.0, 0.0, 0.0),  # Takeoff to 3 meters
            (3.0, 2.0, 0.0),  # Move forward 2 meters
            (3.0, 2.0, -2.5),  # Move downward 2.5 meters
            (3.0, 2.0, 3.0)  # Move up to 3 meters
        ]
        self.movement_counter = 0
        self.repeat_count = 3

    def arm_cb(self, msg):
        self.mode.set_arm()

    def disarm_cb(self, msg):
        self.mode.set_disarm()

    def pos_cb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        q = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        euler = tf2_ros.transformations.euler_from_quaternion(q)
        self.yaw = euler[0]

    def state_cb(self, msg):
        self.state = msg.mode
        self.is_armed = msg.armed

    def landing_state_cb(self, msg):
        self.landed_state = msg.landed_state

    def update(self):
        # Execute the movement sequence
        if self.is_armed and self.landed_state > 1 and self.state == "OFFBOARD":
            target_alt, target_x, target_y = self.movement_sequence[self.movement_counter]
            self.sp.position.z = target_alt
            self.sp.position.x = target_x
            self.sp.position.y = target_y

            self.sp.velocity.z = 0.0  # No vertical velocity for now, just position

            # If we reach the current target, move to the next movement step
            if abs(self.local_pos.z - target_alt) < 0.1:
                self.movement_counter += 1

                # If we've completed one sequence, repeat it if needed
                if self.movement_counter == len(self.movement_sequence):
                    self.movement_counter = 0
                    self.repeat_count -= 1

                # If we've repeated 3 times, disarm the drone
                if self.repeat_count == 0:
                    self.mode.set_auto_land_mode()

        self.sp_pub.publish(self.sp)

    def run(self):
        timer = self.create_timer(0.1, self.timer_callback)
        rclpy.spin(self)

    def timer_callback(self):
        self.update()


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    controller.mode.set_arm()
    controller.mode.set_offboard_mode()
    controller.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
