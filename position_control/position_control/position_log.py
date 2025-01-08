import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Twist, Vector3, Point
from math import pi
from std_msgs.msg import Bool


class PositionLog(Node):
    def __init__(self):
        super().__init__("gps_subscriber")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.curr_alt = 0.0
        self.curr_lat_2 = 0.0
        self.curr_lon_2 = 0.0
        self.curr_alt_2 = 0.0
        self.curr_lat_3 = 0.0
        self.curr_lon_3 = 0.0
        self.curr_alt_3 = 0.0
        self.master_true_yaw = 0.0
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_position_z = 0.0
        self.current_position_x_2 = 0.0
        self.current_position_y_2 = 0.0
        self.current_position_z_2 = 0.0
        self.current_position_x_3 = 0.0
        self.current_position_y_3 = 0.0
        self.current_position_z_3 = 0.0
        self.true_yaw = 0.0

        # create gps subscription
        self.global_position = self.create_subscription(
            VehicleGlobalPosition,
            "/px4_1/fmu/out/vehicle_global_position",
            self.vehicle_global_position_callback,
            qos_profile,
        )
        self.global_position = self.create_subscription(
            VehicleGlobalPosition,
            "/px4_2/fmu/out/vehicle_global_position",
            self.vehicle_global_position_2_callback,
            qos_profile,
        )
        self.global_position = self.create_subscription(
            VehicleGlobalPosition,
            "/px4_3/fmu/out/vehicle_global_position",
            self.vehicle_global_position_3_callback,
            qos_profile,
        )

        self.master_attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/px4_1/fmu/out/vehicle_attitude",
            self.master_attitude_callback,
            qos_profile,
        )

        self.local_position_sub = self.create_subscription(
            VehicleOdometry,
            "/px4_1/fmu/out/vehicle_odometry",
            self.master_odometry_callback,
            qos_profile,
        )
        self.local_position_sub = self.create_subscription(
            VehicleOdometry,
            "/px4_2/fmu/out/vehicle_odometry",
            self.slave2_odometry_callback,
            qos_profile,
        )
        self.local_position_sub = self.create_subscription(
            VehicleOdometry,
            "/px4_3/fmu/out/vehicle_odometry",
            self.slave3_odometry_callback,
            qos_profile,
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def vehicle_global_position_callback(self, msg):
        self.curr_lat = msg.lat
        self.curr_lon = msg.lon
        self.curr_alt = msg.alt
    def vehicle_global_position_2_callback(self, msg):
        self.curr_lat_2 = msg.lat
        self.curr_lon_2 = msg.lon
        self.curr_alt_2 = msg.alt
    def vehicle_global_position_3_callback(self, msg):
        self.curr_lat_3 = msg.lat
        self.curr_lon_3 = msg.lon
        self.curr_alt_3 = msg.alt
    

    def master_attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.master_true_yaw = -(np.arctan2(2.0*(orientation_q[0]*orientation_q[3] + orientation_q[1]*orientation_q[2]), 
                            1.0 - 2.0*(orientation_q[2]*orientation_q[2] + orientation_q[3]*orientation_q[3])))


    def master_odometry_callback(self, msg):
        # 로컬 좌표계에서의 드론 위치를 받아 저장
        self.current_position_x = msg.position[0]
        self.current_position_y = msg.position[1]
        self.current_position_z = msg.position[2]
    def slave2_odometry_callback(self, msg):
        # 로컬 좌표계에서의 드론 위치를 받아 저장
        self.current_position_x_2 = msg.position[0]
        self.current_position_y_2 = msg.position[1]
        self.current_position_z_2 = msg.position[2]
    def slave3_odometry_callback(self, msg):
        # 로컬 좌표계에서의 드론 위치를 받아 저장
        self.current_position_x_3 = msg.position[0]
        self.current_position_y_3 = msg.position[1]
        self.current_position_z_3 = msg.position[2]

    def cmdloop_callback(self):
        self.get_logger().info(f"master_Latitude: {self.curr_lat}")
        self.get_logger().info(f"master_Longitude: {self.curr_lon}")
        self.get_logger().info(f"master_Altitude: {self.curr_alt}")
        self.get_logger().info(f"master_Yaw: {self.master_true_yaw}")
        self.get_logger().info(f"master_x: {self.current_position_x}")
        self.get_logger().info(f"master_y: {self.current_position_y}")
        self.get_logger().info(f"master_z: {self.current_position_z}")
        self.get_logger().info(f"slave1_Latitude: {self.curr_lat_2}")
        self.get_logger().info(f"slave1_Longitude: {self.curr_lon_2}")
        self.get_logger().info(f"slave1_Altitude: {self.curr_alt_2}")
        self.get_logger().info(f"slave1_x: {self.current_position_x_2}")
        self.get_logger().info(f"slave1_y: {self.current_position_y_2}")
        self.get_logger().info(f"slave1_z: {self.current_position_z_2}")
        self.get_logger().info(f"slave2_Latitude: {self.curr_lat_3}")
        self.get_logger().info(f"slave2_Longitude: {self.curr_lon_3}")
        self.get_logger().info(f"slave2_Altitude: {self.curr_alt_3}")
        self.get_logger().info(f"slave2_x: {self.current_position_x_3}")
        self.get_logger().info(f"slave2_y: {self.current_position_y_3}")
        self.get_logger().info(f"slave2_z: {self.current_position_z_3}")


def main(args=None):
    rclpy.init(args=args)
    gps_position_log = PositionLog()

    rclpy.spin(gps_position_log)

    gps_position_log.destroy_node()
    rclpy.shutdown


if __name__ == "__main__":
    main()
