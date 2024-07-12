import sys

import rclpy
from example_interfaces.msg import UInt16MultiArray

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
q : Motor 1 brake start/stop
w : Motor 2 brake start/stop
e : Motor 3 brake start/stop
r : Motor 4 brake start/stop

a/z : Braking strength of motor 1
s/x : Braking strength of motor 2
d/c : Braking strength of motor 3
f/v : Braking strength of motor 4


CTRL-C to quit and Release the brake
"""

moveBindings = {
    'a': (25),
    'z': (-25),
    's': (25),
    'x': (-25),
    'd': (25),
    'c': (-25),
    'f': (25),
    'v': (-25),
    'q': (1),
    'w': (2),
    'e': (3),
    'r': (4)
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


def show_information(m1, m2, m3, m4, s1, s2, s3, s4):
    print('M1: {} ({}), M2: {} ({}), M3: {} ({}), M4: {} ({})'.format(m1, s1, m2, s2, m3, s3, m4, s4))


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('braking_teleop_keyboard')
    pub = node.create_publisher(UInt16MultiArray, 'braking_control', 10)

    m1 = 25, m2 = 25, m3 = 25, m4 = 25
    s1 = 0, s2 = 0, s3 = 0, s4 = 0
    status = 0.0

    try:
        print(msg)
        show_information(m1, m2, m3, m4, s1, s2, s3, s4)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                if key == 'a' or key == 'z':
                    m1 = round(m1 + moveBindings[key])
                    if m1 > 100:
                        m1 = 100
                    if m1 < 0:
                        m1 = 0
                elif key == 's' or key == 'x':
                    m2 = round(m2 + moveBindings[key])
                    if m2 > 100:
                        m2 = 100
                    if m2 < 0:
                        m2 = 0
                elif key == 'd' or key == 'c':
                    m3 = round(m3 + moveBindings[key])
                    if m3 > 100:
                        m3 = 100
                    if m3 < 0:
                        m3 = 0
                elif key == 'f' or key == 'v':
                    m4 = round(m4 + moveBindings[key])
                    if m4 > 100:
                        m4 = 100
                    if m4 < 0:
                        m4 = 0
                elif key == 'q':
                    if s1 == 0:
                        s1 = 1
                    else:
                        s1 = 0
                elif key == 'w':
                    if s2 == 0:
                        s2 = 1
                    else:
                        s2 = 0
                elif key == 'e':
                    if s3 == 0:
                        s3 = 1
                    else:
                        s3 = 0
                elif key == 'r':
                    if s4 == 0:
                        s4 = 1
                    else:
                        s4 = 0

                show_information(m1, m2, m3, m4, s1, s2, s3, s4)

                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

            else:
                m1 = 25, m2 = 25, m3 = 25, m4 = 25
                s1 = 0, s2 = 0, s3 = 0, s4 = 0
                if (key == '\x03'):
                    print("out/out/out/out/out/out/out/out/out/out/out/out/out/out/out/out")
                    break

            braking_control_msg = UInt16MultiArray()
            braking_control_msg.data = [m1, m2, m3, m4, s1, s2, s3, s4]
            pub.publish(braking_control_msg)

    except Exception as e:
        print(e)

    finally:
        m1 = 25, m2 = 25, m3 = 25, m4 = 25
        s1 = 0, s2 = 0, s3 = 0, s4 = 0
        braking_control_msg = UInt16MultiArray()
        braking_control_msg.data = [m1, m2, m3, m4, s1, s2, s3, s4]
        pub.publish(braking_control_msg)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
