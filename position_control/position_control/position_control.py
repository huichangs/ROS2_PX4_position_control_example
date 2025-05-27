#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

import rclpy
import socket
from rclpy.node import Node
import numpy as np
import math
import random
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from geometry_msgs.msg import Twist, Vector3, Point
from std_msgs.msg import Bool



class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #!!!!!!!!!!!!! check and set num of drones!!!!!!!!!!!!!
        self.num_of_drones = 3

        #Create subscriptions
        self.master_status_sub = self.create_subscription(
            VehicleStatus,
            '/px4_1/fmu/out/vehicle_status_v1',
            self.master_vehicle_status_callback,
            qos_profile)
        
        self.master_attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/px4_1/fmu/out/vehicle_attitude',
            self.master_attitude_callback,
            qos_profile)
        
        #create subscriptions created by control.py
        self.offboard_position_sub = self.create_subscription(
            Point,
            '/offboard_position_cmd',
            self.offboard_position_callback,
            qos_profile)
        
        # self.offboard_angular_sub = self.create_subscription(
        #     Twist,
        #     '/offboard_angular_cmd',
        #     self.offboard_angular_callback,
        #     qos_profile)   
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile)
        
        self.my_err_sub = self.create_subscription(
            Bool,
            '/err_message',
            self.err_message_callback,
            qos_profile)
        
        self.my_random_sub = self.create_subscription(
            Bool,
            '/random_message',
            self.random_message_callback,
            qos_profile)
        
        self.master_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position',
            self.master_local_position_callback,
            qos_profile)


        #Create publishers(master drone)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, '/px4_1/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/px4_1/fmu/in/vehicle_command", 10)
        self.err_event_publisher = self.create_publisher(ActuatorMotors, '/px4_1/fmu/in/actuator_motors', qos_profile)

        self.slave_publishers_offboard_mode = {}
        self.slave_publishers_vehicle_command = {}
        #create publishers(slave drones)
        for i in range(2, self.num_of_drones + 1):
            slave_topic_offboard_mode = f'/px4_{i}/fmu/in/offboard_control_mode'
            slave_topic_vehicle_command = f'/px4_{i}/fmu/in/vehicle_command'
            
            self.slave_publishers_offboard_mode[i] = self.create_publisher(OffboardControlMode, slave_topic_offboard_mode, qos_profile)
            self.slave_publishers_vehicle_command[i] = self.create_publisher(VehicleCommand, slave_topic_vehicle_command, qos_profile)
        
        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(0.2, self.cmdloop_callback)
        self.timer = self.create_timer(0.001, self.errloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        
        self.offboardMode = False
        self.flightCheck = False
        self.arm_message = False
        self.err_event = False
        self.random_event = False
        self.failsafe = False
        
        self.target_position_x = 0
        self.target_position_y = 0
        self.target_position_z = 3
        
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_position_z = 0.0
        self.myCnt = 0
        self.yaw = 0.0  #yaw value we send as command
        self.local_yaw = 0.0
        self.trueYaw = 0.0  #current yaw value of drone
        
        self.last_position_update_time = self.get_clock().now().seconds_nanoseconds()[0]

        #states with corresponding callback functions that run once when state switches
        self.states = {
            "IDLE": self.state_init,
            "ARMING": self.state_arming,
            "TAKEOFF": self.state_takeoff,
            "LOITER": self.state_loiter,
            "OFFBOARD": self.state_offboard
        }
        self.current_state = "IDLE"
        self.last_state = self.current_state

        # socket setup
        # 서버 주소와 포트 설정
        self.UDP_IP = "127.0.0.1"   # 로컬호스트
        self.UDP_PORT = 9999

        # 소켓 생성
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):

        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 5):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_init(self):
        self.myCnt = 0

    def state_arming(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 1.0)
        for i in range(2, self.num_of_drones + 1):
            self.publish_to_slave(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, i, 1.0)
        
        self.get_logger().info("Arm command send")

    def state_takeoff(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 1, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        for i in range(2, self.num_of_drones + 1):
            self.publish_to_slave(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, i, param1 = 1.0, param7=5.0)
        self.get_logger().info("Takeoff command send")

    def state_loiter(self):
        self.myCnt = 0
        self.get_logger().info("Loiter Status")

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 1., 6.)
        for i in range(2, self.num_of_drones + 1):
            self.publish_to_slave(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, i, 1., 6.)
        self.offboardMode = True

        

    # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 1.0)
        for i in range(2, self.num_of_drones + 1):
            self.publish_to_slave(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, i, 1.0)
        self.get_logger().info("Arm command send")

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 1, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        for i in range(2, self.num_of_drones + 1):
            self.publish_to_slave(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, i, param1 = 1.0, param7=5.0)
        self.get_logger().info("Takeoff command send")
        
        
        
    
    #receives and sets vehicle status values 
    def master_vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass


    #get from control.py
    def offboard_position_callback(self, msg):
        self.target_position_x = msg.x
        self.target_position_y = msg.y
        self.target_position_z = msg.z
        
    def offboard_angular_callback(self, msg):
        self.yaw = msg.angular.z

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def master_attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                            1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
    
    def master_local_position_callback(self, msg):
        # 로컬 좌표계에서의 드론 위치를 받아 저장
        self.current_position_x = msg.x
        self.current_position_y = msg.y
        self.current_position_z = msg.z

    def err_message_callback(self, msg):
        self.err_event = msg.data

    def random_message_callback(self, msg):
        self.random_event = msg.data
    
    
    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, target_system, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = target_system + 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    #publishes command to slave drones
    def publish_to_slave(self, command, slave_id, param1=0.0, param2=0.0, param7=0.0):
        if slave_id in self.slave_publishers_vehicle_command:
            msg = VehicleCommand()
            msg.param1 = param1
            msg.param2 = param2
            msg.param7 = param7
            msg.command = command
            msg.target_system = slave_id + 1  # Adjust as needed
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(Clock().now().nanoseconds / 1000)
            self.slave_publishers_vehicle_command[slave_id].publish(msg)
            
    def errloop_callback(self):
        if(self.err_event == True):
            self.get_logger().info(f"err_event : {self.err_event}")
            motor_msg = ActuatorMotors()
            motor_msg.timestamp = int(Clock().now().nanoseconds / 1000)
 
            motor_msg.control[1] = float('nan')
            
            self.err_event_publisher.publish(motor_msg)
            
            self.sock.sendto(b"True", (self.UDP_IP, self.UDP_PORT))
        else:
            self.sock.sendto(b"False", (self.UDP_IP, self.UDP_PORT))
    
    
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if self.offboardMode == True:
            now_sec = self.get_clock().now().seconds_nanoseconds()[0]
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)
            for i in range(2, self.num_of_drones + 1):
                self.slave_publishers_offboard_mode[i].publish(offboard_msg)

            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            if self.random_event == True:
                self.current_position_x = -self.current_position_x
                dx = -(self.target_position_x - (self.current_position_x))
                dy = self.target_position_y - self.current_position_y

                    # atan2를 사용하여 목표 위치를 향한 각도 계산
                distance = math.sqrt(dx**2 + dy**2)
                if distance > 0.3:  # 목표 위치와 현재 위치의 거리가 0.1m 이상일 때만 yaw 업데이트
                    desired_yaw = math.atan2(dy, dx)
                else:
                    desired_yaw = self.yaw

                # 업데이트된 yaw 값을 설정
                self.yaw = desired_yaw

                if now_sec - self.last_position_update_time >= 5:
                    self.target_position_x = random.uniform(-1000, 1000)
                    self.target_position_y = random.uniform(-1000, 1000)
                    self.target_position_z = -random.uniform(5, 100)  # NED 기준 z는 음수
                    self.last_position_update_time = now_sec
                    self.get_logger().info(f"[New Target] x={self.target_position_x:.2f}, y={self.target_position_y:.2f}, z={self.target_position_z:.2f}")

                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                    trajectory_msg.position[0] = self.target_position_x
                    trajectory_msg.position[1] = self.target_position_y
                    trajectory_msg.position[2] = self.target_position_z
                    trajectory_msg.yaw = float(desired_yaw)
                    self.publisher_trajectory.publish(trajectory_msg)
            else:
                self.current_position_x = -self.current_position_x
                dx = -(self.target_position_x - (self.current_position_x))
                dy = self.target_position_y - self.current_position_y

                # atan2를 사용하여 목표 위치를 향한 각도 계산
                distance = math.sqrt(dx**2 + dy**2)
                if distance > 0.3:  # 목표 위치와 현재 위치의 거리가 0.1m 이상일 때만 yaw 업데이트
                    desired_yaw = math.atan2(dy, dx)
                else:
                    desired_yaw = self.yaw

                
                # 업데이트된 yaw 값을 설정
                self.yaw = desired_yaw
                
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                trajectory_msg.velocity[0] = float('nan')
                trajectory_msg.velocity[1] = float('nan')
                trajectory_msg.velocity[2] = float('nan')
                trajectory_msg.position[0] = -self.target_position_x
                trajectory_msg.position[1] = self.target_position_y
                trajectory_msg.position[2] = -self.target_position_z
                trajectory_msg.acceleration[0] = float('nan')
                trajectory_msg.acceleration[1] = float('nan')
                trajectory_msg.acceleration[2] = float('nan')
                trajectory_msg.yaw = float(desired_yaw)

                self.publisher_trajectory.publish(trajectory_msg)



def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()