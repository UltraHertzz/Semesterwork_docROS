# if Warning is shownd, use command to set environment variable: export PATH="/home/orangepi/.local/bin:$PATH"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
import time


class MuxNode(Node):
    """
    Mux node that receive String message from keyboard 
    to switch the control source from (key_board, joy_stick, algorithms...)

    - Subscribe
        - vel_cmd:
            - Published from: Keyboard, JoyStick, Nav2, ...
            - Type: Float64MuliArray() [x(vel_angular), y(vel_linear), t(time)] 
            - Description: control signals to make vehicle turn or go straight line(not that straight)
        - mode:
            - Published from: Keyboard
            - Type: String() mode 
            - Description: Change mode between different control source
    
    - Publish(Car Drive)
        - vel_cmd:
            - Publish to: Car Drive(by timer)
            - type: Float64MuliArray [x(vel_angular), y(vel_linear), t(time)]
    """
    
    def __init__(self):

        super().__init__('mux_node')

        # sub (js, kb, al, ...) 1 ... N
        self.js_sub = self.create_subscription(Float64MultiArray, '/controller/joy_stick', self.js_control, 10)
        self.kb_sub = self.create_subscription(Float64MultiArray, '/controller/key_board', self.kb_control, 10)
        self.al_sub = self.create_subscription(Twist, '/cmd_vel', self.al_control, 10)
        self.mode_sub = self.create_subscription(String, 'controller/mode', self.callback, 10)
        self.timer_pub = self.create_timer(0.02, self.timer_callback) # 50Hz control signal

        # pub (only to driver)
        self.drive_pub = self.create_publisher(Float64MultiArray, '/controller/mux', 10)

        # msg & mode
        self.mode = 'j'
        self.kb_msg = Float64MultiArray()
        self.js_msg = Float64MultiArray()
        self.al_msg = Float64MultiArray()

    def kb_control(self, msg):

        self.kb_msg.data = msg.data
        
    def js_control(self, msg):

        self.js_msg.data = msg.data

        # test
        # self.drive_pub.publish(self.js_msg)

    def al_control(self, msg):
        
        # subscribe /cmd_vel( type : geometry_msgs/msg/Twist )
        al_y = msg.data.linear.x
        al_x = msg.data.angular.z
        al_t = time.time()
        al_msg.data = [al_x, al_y, al_t]
        self.al_msg.data = msg.data

    def callback(self, msg):

        """
        call back function of mode, switch in different control mode: Keyboard, Joystick, Algorithm
        """
        # change self.mode
        if msg.data == 'j': # using joystick
            self.mode = 'j'

        if msg.data == 'k':
            self.mode = 'k'

        if msg.data == 'l':
            self.mode = 'l'

        # print(self.mode)

    def timer_callback(self):

        # publish related msg
        if self.mode == 'k':
            self.get_logger().info('Control with KeyBoard!')
            self.drive_pub.publish(self.kb_msg)

        if self.mode == 'j':
            self.get_logger().info('Control with JoyStick!')
            self.drive_pub.publish(self.js_msg)

        if self.mode == 'l':
            self.get_logger().info('Control with Algorithm!')
            self.drive_pub.publish(self.al_msg)

def main(args=None):
    rclpy.init(args=args)
    muxNode = MuxNode()
    try:
        while rclpy.ok():
            rclpy.spin(muxNode)
    except KeyboardInterrupt:
        pass
    finally:
        muxNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
