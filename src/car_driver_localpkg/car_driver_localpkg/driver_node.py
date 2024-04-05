import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from car_driver_localpkg.gpio_init import CarController_SideTurn, CarController_CenterTurn
import time

class CarDriver(Node):

    """
    Publication: 
        - topic name: /exec_time
        - type: Float64
        - description: watch the execution time of one control command being implemented by actuator in one call

    Subscription:
        - topic name: /mux 
        - from Node: mux
        - type: Float64MultiArray [x,y,t]
        - description: subscribe to control value that published by mux node and drive the car
    """

    def __init__(self):

        super().__init__('driver')

        self.sub = self.create_subscription(Float64MultiArray, 
                                            '/controller/mux',self.call_back, 10)
        self.pub = self.create_publisher(Float64, '/exec_time', 10)
        
        # Set turning mode (differential turning || axial turning)
        # self.car = CarController_SideTurn() # Mode 1. diff turning (continous turning)
        self.car = CarController_CenterTurn() # Mode 2. axial turning (turning separately)

        self.x = 0.0
        self.y = 0.0
        self.t = 0.0

    """
    # This snippet is used for mode differential turning (Mode 1), uncomment when use Mode 1
    def call_back(self, msg):

        x, y, t = msg.data

        if y > 0:
            self.car.drive_forward(x,y,100)
            self.car.recover()
        elif y < 0:
            self.car.drive_back(x,y,100)
            self.car.recover()
        else:
            self.car.stop()
        delta_t = (time.time() - t)
        #self.get_logger().info('Execution Time: "%s"' % delta_t)
        pub_msg = Float64()
        pub_msg.data = delta_t
        self.pub.publish(pub_msg)
    """

    # This snippet is used for mode axial turning (Mode 2), uncomment when use Mode 2
    def call_back(self, msg):

        self.x, self.y, self.t = msg.data
        if self.x > 0:
            print("tern right")
            self.car.drive_right(self.x,100)
            self.car.recover()
        elif self.x < 0:
            print("turn left")
            self.car.drive_left(self.x,100)
            self.car.recover()
        else:
            if self.y > 0:
                self.car.drive_forward(self.y,100)
                self.car.recover()
            elif self.y < 0:
                self.car.drive_back(self.y,100)
                self.car.recover()
            else:
                self.car.stop()
        delta_t = (time.time() - self.t)
        #self.get_logger().info('Execution Time: "%s"' % delta_t)
        pub_msg = Float64()
        pub_msg.data = delta_t
        self.pub.publish(pub_msg)
        
def main(args=None):

    rclpy.init(args=args)
    driver = CarDriver()
    rclpy.spin(driver)

    driver.car.exit() # clean up all gpios
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()