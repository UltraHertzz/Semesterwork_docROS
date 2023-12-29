import keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class ControlPublisher(Node):

    def __init__(self) -> None:
        super().__init__("game_pad")
        self.publisher = self.create_publisher(String, '/controller/key_board', 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.stop_flag = False

    def timer_callback(self):
        msg = Float32MultiArray()
        control_value = input()
        match control_value:

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
        msg.data = [x,y]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    print('Publish Joystick Control Value')
    pub = ControlPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
