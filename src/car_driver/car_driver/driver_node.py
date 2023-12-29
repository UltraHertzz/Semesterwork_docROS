import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from car_driver.gpio_init import CarController

class CarDriver(Node):
    """
    Publication: None
    Subscription: /mux (Float32MultiArray [x,y] which is the control value in direction of [])
    Description: subscribe to control value that published by mux node and drive the car
    """
    def __init__(self):

        super().__init__('driver')

        self.sub = self.create_subscription(Float32MultiArray, 
                                            '/controller/mux',self.call_back, 10)
        self.car = CarController()

    def call_back(self, msg):

        x, y = msg.data

        if y > 0:
            self.car.recover()
            self.car.drive_forward(x,y,100)
        elif y < 0:
            self.car.recover()
            self.car.drive_back(x,y,100)
        else:
            self.car.stop()
        
def main(args=None):

    rclpy.init(args=args)
    driver = CarDriver()
    rclpy.spin(driver)

    driver.car.exit() # clean up all gpios
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()