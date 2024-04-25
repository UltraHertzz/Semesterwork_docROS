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
            - Published from: JoyStick, Nav2, ...
            - Type: Float64MuliArray() [x(vel_angular), y(vel_linear), t(time)] 
            - Description: control signals to make vehicle turn or go straight line(not that straight)
        - mode:
            - Published from: Gamepad
            - Type: String() mode 
            - Description: Change mode between different control source
    
    - Publish(Car Drive)
        - vel_cmd:
            - Publish to: Car Drive(by timer)
            - type: Float64MuliArray [x(vel_angular), y(vel_linear), t(time)]
    """
    
    def __init__(self):

        super().__init__('mux_node')

        # self.signals = {}
        # self.subscribers = {
            # "j": self.create_subscription(Float64MultiArray, '/controller/joy_stick', self.js_control, 10),
            # "k": self.create_subscription(Float64MultiArray, '/controller/key_board', self.kb_control, 10),
            # "l": self.create_subscription(Twist, '/cmd_vel', self.al_control, 10)
        # }
        
        # sub (js, kb, al, ...) 1 ... N
        self.js_sub = self.create_subscription(Float64MultiArray, '/controller/joy_stick', self.js_control, 10)
        self.kb_sub = self.create_subscription(Float64MultiArray, '/controller/key_board', self.kb_control, 10)
        self.al_sub = self.create_subscription(Twist, '/cmd_vel', self.al_control, 10)
        self.mode_sub = self.create_subscription(String, 'controller/mode', self.callback, 10)

        # pub (only to driver)
        self.drive_pub = self.create_publisher(Float64MultiArray, '/controller/mux', 10)

        # msg & mode
        self.mode = 'j'
        self.kb_msg = Float64MultiArray()
        self.js_msg = Float64MultiArray()
        self.al_msg = Float64MultiArray()

    def kb_control(self, msg):

        self.kb_msg.data = msg.data
        if self.mode == "k":
            self.drive_pub.publish(self.kb_msg)
        
    def js_control(self, msg):

        self.js_msg.data = msg.data
        if self.mode == "j":
            self.drive_pub.publish(self.js_msg)

        # test
        # self.drive_pub.publish(self.js_msg)

    def al_control(self, msg):
        
        # subscribe /cmd_vel( type : geometry_msgs/msg/Twist )
        al_y = msg.linear.x
        al_x = msg.angular.z * 0.25
        al_t = time.time()
        self.al_msg.data = [al_x, al_y, al_t]
        if self.mode == "l":
            self.drive_pub.publish(self.al_msg)

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
