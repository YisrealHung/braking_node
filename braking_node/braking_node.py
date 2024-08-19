import serial
import time

import rclpy
from rclpy.node import Node
from example_interfaces.msg import UInt16MultiArray


class Braking(Node):

    def __init__(self, port = '/dev/braking',baud = 9600):
        super().__init__('braking_node')
        self.subscription = self.create_subscription(UInt16MultiArray, 'braking_control', self.cmd_braking_callback, 10)
        self.ser = serial.Serial(port, baud, timeout = 1)
        time.sleep(5)
        return_data = self.ser.readline().decode('utf-8').strip()
        self.get_logger().info('Braking node is working...')


    # 19 32 4B 64
    def change_current_to_cmd(self, c):
        if c == 25:
            return bytearray(b'\x19')
        elif c == 50:
            return bytearray(b'\x32')
        elif c == 75:
            return bytearray(b'\x4B')
        elif c == 100:
            return bytearray(b'\x64')
        else:
            return bytearray(b'\x00')

    def braking_action(self, m1, m2, m3, m4):
        set_braking_current = bytearray(b'\x7b')
        set_braking_current += self.change_current_to_cmd(m1)
        set_braking_current += self.change_current_to_cmd(m2)
        set_braking_current += self.change_current_to_cmd(m3)
        set_braking_current += self.change_current_to_cmd(m4)
        set_braking_current += bytearray(b'\x7d')
        self.ser.write(set_braking_current)
        return_data = self.ser.readline().decode('utf-8').strip()
        self.ser.write(set_braking_current)
        return_data = self.ser.readline().decode('utf-8').strip()
        print(return_data)


    def cmd_braking_callback(self, msg):
        m1, m2, m3, m4, s1, s2, s3, s4 = msg.data
        if s1 == 0:
            m1 = 0
        if s2 == 0:
            m2 = 0
        if s3 == 0:
            m3 = 0
        if s4 == 0:
            m4 = 0

        self.braking_action(m1, m2, m3, m4)



def main(args=None):
    rclpy.init(args=args)
    node = Braking(port = '/dev/braking',baud = 9600)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
