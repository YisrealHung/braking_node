import sys

import rclpy
from example_interfaces.msg import UInt16MultiArray
import geometry_msgs.msg

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
t : Motor 2 turn on/off
g/b : increase/decrease Linear velocity in x by 10%

q : Motor 2 brake start/stop
a/z : Braking strength of motor

CTRL-C to quit and Release the brake
"""

moveBindings = {
    'a': (25),
    'z': (-25),
    'g': (0.05),
    'b': (-0.05),
    'q': (1),
    't': (2)
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
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


def show_information(m2, s2, sp1):
    if s2 == 1:
        s2 = "ON"
    else:
        s2 = "OFF"

    print('M2: {}% ({}) speed: {}'.format(m2, s2, sp1))


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('braking_teleop_keyboard')
    pub = node.create_publisher(UInt16MultiArray, 'braking_control', 10)
    pub_cmd = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

    m1 = 25
    m2 = 25
    m3 = 25
    m4 = 25
    s1 = 1
    s2 = 0
    s3 = 1
    s4 = 1
    status = 0.0
    sp1 = 0.1
    t1 = 0

    try:
        print(msg)
        show_information(m2, s2, sp1)
        
        for i in range(2):
            braking_control_msg = UInt16MultiArray()
            braking_control_msg.data = [25, 25, 25, 25, 1, 0, 1, 1]
            pub.publish(braking_control_msg)
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub_cmd.publish(twist)

        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                if key == 'a' or key == 'z':
                    m2 = round(m2 + moveBindings[key])
                    if m2 > 100:
                        m2 = 100
                    if m2 < 0:
                        m2 = 0

                elif key == 'q':
                    if s2 == 0:
                        s2 = 1
                    else:
                        s2 = 0

                if key == 'g' or key == 'b':
                    sp1 = sp1 + moveBindings[key]

                elif key == 't':
                    if t1 == 0:
                        t1 = 1
                    else:
                        t1 = 0

                show_information(m2, s2, sp1)

                if (status == 14):
                    print(msg)
                    show_information(m2, s2, sp1)
                status = (status + 1) % 15

            else:
                m1 = 25
                m2 = 25
                m3 = 25
                m4 = 25
                s1 = 1
                s2 = 0
                s3 = 1
                s4 = 1
                sp1 = 0.1
                t1 = 0
                show_information(m2, s2, sp1)

                if (key == '\x03'):
                    braking_control_msg = UInt16MultiArray()
                    braking_control_msg.data = [25, 25, 25, 25, 1, 0, 1, 1]
                    pub.publish(braking_control_msg)
                    twist = geometry_msgs.msg.Twist()
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                    pub_cmd.publish(twist)
                    break

            braking_control_msg = UInt16MultiArray()
            braking_control_msg.data = [m1, m2, m3, m4, s1, s2, s3, s4]
            pub.publish(braking_control_msg)

            if (t1 == 1):
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = sp1
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                pub_cmd.publish(twist)
            else:
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                pub_cmd.publish(twist)


    except Exception as e:
        print(e)

    finally:
        m1 = 25
        m2 = 25
        m3 = 25
        m4 = 25
        s1 = 0
        s2 = 0
        s3 = 0
        s4 = 0
        braking_control_msg = UInt16MultiArray()
        braking_control_msg.data = [m1, m2, m3, m4, s1, s2, s3, s4]
        pub.publish(braking_control_msg)

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub_cmd.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
