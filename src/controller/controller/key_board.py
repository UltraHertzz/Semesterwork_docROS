import time
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray

class ControlPublisher(Node):

    def __init__(self) -> None:
        super().__init__("game_pad")
        self.control_pub = self.create_publisher(Float64MultiArray, '/controller/key_board', 10)
        self.mode_pub = self.create_publisher(String, '/controller/mode',1)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mode = 'j'
        self.t = time.time()
        self.control_val = [0, 0, self.t] # x,y,t
        self.stop_flag = False
        self.spd_limit = 0.25

    def get_key_press(self):
        # get key input
        fd = sys.stdin.fileno()
        og_attr = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, og_attr)
        return key

    def timer_callback(self):
        # publish 
        con_msg = Float32MultiArray()
        mode_msg = String()
        control_value = self.get_key_press()
        mode_value = control_value
        match control_value:

            # control value 
            case 'w':
                self.control_val = [0, 1*self.spd_limit, self.t]
            case 'a':
                self.control_val = [-1*spd_limit, 0, self.t]
            case 's':
                self.control_val = [0, -1*spd_limit, self.t]
            case 'd':
                self.control_val = [1*spd_limit, 0, self.t]
            case _:
                self.control_val = [0.0, 0.0, self.t]

        match mode_value:

            # mode value
            case 'j': # joy stick control
                self.mode = 'j'
            case 'k': # key board control
                self.mode = 'k'


        con_msg.data = self.control_val
        mode_msg.data = self.mode
        self.control_pub.publish(con_msg)
        self.mode_pub.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    print('Publish Keyboard Control Value')
    pub = ControlPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
