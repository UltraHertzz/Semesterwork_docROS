# if Warning is shownd, use command to set environment variable: export PATH="/home/orangepi/.local/bin:$PATH"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time


class MuxNode(Node):
    """
    Mux node that receive Char message from keyboard 
    to switch the control source from (key_board, joy_stick, algorithms...)
    """
    
    def __init__(self):

        super().__init__('mux_node')
        self.js_sub = self.create_subscription(Float32MultiArray, '/controller/joy_stick',self.js_control,10)
        self.kb_sub = self.create_subscription(String, '/controller/key_board', self.kb_control, 10)
        self.drive_pub = self.create_publisher(Float32MultiArray, '/controller/mux', 10)
        self.mode = 'j'
        self.kb_msg = Float32MultiArray()
        self.js_msg = Float32MultiArray()

    def kb_control(self, msg):

        match msg.data:

            # control value 
            case 'w':
                (x,y) = (0.0, 0.8)
            case 'a':
                (x,y) = (-0.3, 0.5)
            case 's':
                (x,y) = (0.3, 0.5)
            case 'd':
                (x,y) = (0.0, -0.3)

            # mode value
            case 'j': # joy stick control
                self.mode = 'j'
            case 'k': # key board control
                self.mode = 'k'

            case _:
                (x,y) = (0.0, 0.0)

        self.kb_msg.data = [x,y]
        
    def js_control(self, msg):

        self.js_msg.data = msg.data

    def callback(self):
        """
        call back function of mode, switch in different control mode: Keyboard, Joystick, Algorithm
        """
        if self.mode == 'k':
            #print('Control with Keyboard!')
            self.drive_pub.publish(self.kb_msg)
        if self.mode == 'j':
            #print("Control with Joy Con!")
            self.drive_pub.publish(self.js_msg)

def main(args=None):
    rclpy.init(args=args)
    muxNode = MuxNode()
    try:
        while rclpy.ok():
            muxNode.callback()
            time.sleep(0.02)
            rclpy.spin_once(muxNode, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        muxNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
