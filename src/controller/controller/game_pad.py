""" Xbox 360 controller support for Python
11/9/13 - Steven Jacobs

This class module supports reading a connected xbox controller.
It requires that xboxdrv be installed first:

    sudo apt-get install xboxdrv

See http://pingus.seul.org/~grumbel/xboxdrv/ for details on xboxdrv

Example usage:

    import xbox
    joy = xbox.Joystick()         #Initialize joystick
    
    if joy.A():                   #Test state of the A button (1=pressed, 0=not pressed)
        print 'A button pressed'
    x_axis   = joy.leftX()        #X-axis of the left stick (values -1.0 to 1.0)
    (x,y)    = joy.leftStick()    #Returns tuple containing left X and Y axes (values -1.0 to 1.0)
    trigger  = joy.rightTrigger() #Right trigger position (values 0 to 1.0)
    
    joy.close()                   #Cleanup before exit
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from controller.xbox import Joystick


class ControlPublisher(Node):

    def __init__(self) -> None:
        super().__init__("game_pad")
        self.publisher = self.create_publisher(Float64MultiArray, 
                                               '/controller/joy_stick', 10)
        self.pub_time_period = 0.02  # 50Hz
        self.timer = self.create_timer(self.pub_time_period, self.timer_callback)
        self.joy = Joystick()
        self.stop_flag = False

    def timer_callback(self) -> None:
        # publish controll topic with a fixed rate 
        if self.joy.B():
            self.stop_flag = True
        if self.joy.A():
            self.stop_flag = False

        msg = Float64MultiArray()

        x, y = self.joy.leftStick()
        t = time.time()

        if self.stop_flag:
            msg.data = [0.0, 0.0, t]
        else:
            msg.data = [0.25*x, 0.25*y, t]

        self.publisher.publish(msg)

    """

    def publishment(self) -> None:

        # To publish topic as fast as possible (rate: 4000Hz)

        while rclpy.ok():

            if self.joy.B():
                self.stop_flag = True
            if self.joy.A():
                self.stop_flag = False    
            msg = Float64MultiArray()
            x, y = self.joy.leftStick()
            t = time.time()
            if self.stop_flag:
                msg.data = [0.0, 0.0, t]
            else:
                msg.data = [0.25*x, 0.25*y, t]
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.02)
    """
def main(args=None):
    rclpy.init(args=args)
    print('Publish Joystick Control Value')
    pub = ControlPublisher()

    rclpy.spin(pub)
    # pub.publishment()

    pub.destroy_node()
    rclpy.shutdown()





if __name__ == '__main__':
    main()
