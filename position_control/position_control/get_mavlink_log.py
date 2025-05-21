import csv
import time
from pymavlink import mavutil
from datetime import datetime
import socket

# MAVLink 연결
mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')

# HEARTBEAT 대기 (연결 확인)
mav.wait_heartbeat()
print(f"Connected to system (ID {mav.target_system}, component {mav.target_component})")

# 서버 주소와 포트 설정
UDP_IP = "0.0.0.0"       # 모든 IP로부터 수신
UDP_PORT = 9999

# 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"UDP 서버 실행 중... {UDP_IP}:{UDP_PORT}")

flag = True

while flag:
    # CSV 파일 생성
    log_dir = "/home/ciderlab-server1/ws/src/ROS2_PX4_position_control_example/position_control/log_data"
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{log_dir}/mavlink_log_{timestamp_str}.csv"

    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # CSV 헤더 작성
        writer.writerow([
            'local_time_boot_ms', 'target_time_boot_ms', 'attitude_time_boot_ms', 'servo_time_usec', 
            'x', 'y', 'z', 'vx', 'vy', 'vz', 
            'target_x', 'target_y', 'target_z', 'target_vx', 'target_vy', 'target_vz',
            'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed',
            'servo1_raw', 'servo2_raw', 'servo3_raw', 'servo4_raw',
            'is_err'
        ])
        
        print(f"[INFO] Logging to {filename} ... Press Ctrl+C to stop.")
        
        start_time = time.time()

        try:
            while True:
                local_position_msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True)
                position_target_local_msg = mav.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=True)
                attitude_msg = mav.recv_match(type='ATTITUDE', blocking=True)
                servo_msg = mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
                is_err, addr = sock.recvfrom(1024)
                #print(attitude_msg)
                if attitude_msg and servo_msg:
                    # CSV 파일에 데이터 기록
                    writer.writerow([
                        local_position_msg.time_boot_ms, position_target_local_msg.time_boot_ms, attitude_msg.time_boot_ms, servo_msg.time_usec,
                        local_position_msg.x, local_position_msg.y, local_position_msg.z, local_position_msg.vx, local_position_msg.vy, local_position_msg.vz, 
                        position_target_local_msg.x, position_target_local_msg.y, position_target_local_msg.z, 
                        position_target_local_msg.vx, position_target_local_msg.vy, position_target_local_msg.vz,
                        attitude_msg.roll, attitude_msg.pitch, attitude_msg.yaw, attitude_msg.rollspeed, attitude_msg.pitchspeed, attitude_msg.yawspeed,
                        servo_msg.servo1_raw, servo_msg.servo2_raw, servo_msg.servo3_raw, servo_msg.servo4_raw,
                        is_err.decode()
                    ])
                    
                current_time = time.time()
                if current_time - start_time > 600:
                    break

        except KeyboardInterrupt:
            print("\n[INFO] Logging stopped.")
            flag = False
