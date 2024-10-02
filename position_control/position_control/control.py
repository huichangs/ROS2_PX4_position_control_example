#!/usr/bin/env python3
import sys

import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
"""

moveBindings = {
    'w': (0, 0, 1, 0), #Z+
    's': (0, 0, -1, 0),#Z-
    'a': (0, 0, 0, -1), #Yaw+
    'd': (0, 0, 0, 1),#Yaw-
    '\x1b[A' : (0, 1, 0, 0),  #Up Arrow
    '\x1b[B' : (0, -1, 0, 0), #Down Arrow
    '\x1b[C' : (1, 0, 0, 0), #Right Arrow
    '\x1b[D' : (-1, 0, 0, 0),  #Left Arrow
}


speedBindings = {
    # 'q': (1.1, 1.1),
    # 'z': (.9, .9),
    # 'w': (1.1, 1),
    # 'x': (.9, 1),
    # 'e': (1, 1.1),
    # 'c': (1, .9),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2)  # read the next two characters
            key += additional_chars  # append these characters to the key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


    pub = node.create_publisher(geometry_msgs.msg.Point, '/offboard_position_cmd', qos_profile)
    ang_pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_angular_cmd', qos_profile)
    arm_toggle = False
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)


    speed = 0.5
    turn = 0.02
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 5.0
    yaw_val = 0.0
    destination = None


    try:
        print(msg)
        # print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if key == ' ':  # ASCII value for space
                arm_toggle = not arm_toggle  # Flip the value of arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)
                print(f"Arm toggle is now: {arm_toggle}")

            twist = geometry_msgs.msg.Twist()
            yaw_val = (th * turn) + yaw_val
            if yaw_val > 3.14:
                yaw_val -= 2 * 3.14
            elif yaw_val < -3.14:
                yaw_val += 2 * 3.14
            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = yaw_val
            ang_pub.publish(twist)

            point = geometry_msgs.msg.Point()
            x_val = x + x_val
            y_val = y + y_val
            z_val = z + z_val
            point.x = x_val
            point.y = y_val
            point.z = z_val
            # print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)
            print(f"Position: X: {point.x}, Y: {point.y}, Z: {point.z}, Yaw: {twist.angular.z}")
            pub.publish(point)


    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        ang_pub.publish(twist)
        point = geometry_msgs.msg.Point()
        point.x = 0.0
        point.y = 0.0
        point.z = 0.0
        pub.publish(point)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()