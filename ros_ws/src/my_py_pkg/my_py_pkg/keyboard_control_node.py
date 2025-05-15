import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import CmdDriveVel
import sys, select, termios, tty
import numpy as np

MAX_VEL = 0.5
MAX_OMEGA = 4
ACCEL_STEP = 0.15   
OMEGA_STEP = 0.4   
           

class TeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.vel_pub = self.create_publisher(CmdDriveVel, 'arduino/cmd_vel', 10)

        self.v = 0.0
        self.omega = 0.0

        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.loop)  

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def loop(self):
        key = get_key()
        dt = (self.get_clock().now() - self.prev_time).nanoseconds * 1e-9
        self.prev_time = self.get_clock().now()

        # Process key for control
        if key == '\x1b[A':  # Up
            self.v = min(self.v + ACCEL_STEP, MAX_VEL)
        elif key == '\x1b[B':  # Down
            self.v = max(self.v - ACCEL_STEP, -MAX_VEL)
        elif key == '\x1b[C':  # Right
            self.omega = max(self.omega - OMEGA_STEP, -MAX_OMEGA)
        elif key == '\x1b[D':  # Left
            self.omega = min(self.omega + OMEGA_STEP, MAX_OMEGA)
        elif key == ' ':  # Space = stop
            self.v = 0.0
            self.omega = 0.0
        else:
            self.v = 0.0
            self.omega = 0.0


        msg = CmdDriveVel()       
        msg.vx = float(self.v) 
        msg.omega = float(-self.omega)
        
        self.vel_pub.publish(msg)


       

def get_key():
    dr, _, _ = select.select([sys.stdin], [], [], 0.01)
    if dr:
        key = sys.stdin.read(1)
        if key == '\x1b':
            return key + sys.stdin.read(2)  # handle arrow keys
        return key
    return ''

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
