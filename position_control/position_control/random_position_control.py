import rclpy
import random
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/px4_1/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_profile)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.current_position = [0.0, 0.0, -10.0]  # 초기 좌표 (z는 음수)
        self.last_position_update_time = self.get_clock().now().seconds_nanoseconds()[0]

 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state
        

    def cmdloop_callback(self):
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]

        # 5초마다 새로운 랜덤 좌표 생성
        if now_sec - self.last_position_update_time >= 5:
            x = random.uniform(-1000, 1000)
            y = random.uniform(-1000, 1000)
            z = -random.uniform(5, 100)  # NED 기준 z는 음수
            self.current_position = [x, y, z]
            self.last_position_update_time = now_sec
            self.get_logger().info(f"[New Target] x={x:.2f}, y={y:.2f}, z={z:.2f}")

        # 오프보드 모드 퍼블리시
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # Trajectory 퍼블리시
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = self.current_position[0]
            trajectory_msg.position[1] = self.current_position[1]
            trajectory_msg.position[2] = self.current_position[2]
            self.publisher_trajectory.publish(trajectory_msg)
        


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()