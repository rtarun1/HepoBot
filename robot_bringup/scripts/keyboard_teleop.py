#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x
w : move forward (0.4 m/s)
x : move backward (-0.4 m/s)
a : turn left (0.5 rad/s)
d : turn right (-0.5 rad/s)
s/space : stop
CTRL-C to quit
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        print(msg)

    def get_key(self):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()

                if key == 'w':
                    twist.linear.x = 0.2
                elif key == 'x':
                    twist.linear.x = -0.2
                elif key == 'a':
                    twist.angular.z = 0.2
                elif key == 'd':
                    twist.angular.z = -0.2
                elif key in ('s', ' '):
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03':  # CTRL+C
                    break
                else:
                    continue  # Do not publish anything if no key is pressed

                self.publisher.publish(twist)
                print(f"Linear Vel: {twist.linear.x:.2f}, Angular Vel: {twist.angular.z:.2f}")

        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()
            rclpy.shutdown()
            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    rclpy.init()
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    teleop_node = KeyboardTeleop()
    teleop_node.run()
