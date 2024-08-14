import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from car_driver_localpkg.gpio_init import CarController_Mecanum
import time
import numpy as np

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
        self.pub = self.create_publisher(Float64, '/exec_time/controller_to_driver', 10)
        
        # Set turning mode (differential turning || axial turning)
        self.car = CarController_Mecanum() # Mode 1. diff turning (continous turning)
        # self.car = CarController_CenterTurn() # Mode 2. axial turning (turning separately)

        self.x = 0.0
        self.y = 0.0
        self.xr = 0.0
        self.t = 0.0
        self.spd_limit = 1.0

    
    # This snippet is used for mode differential turning (Mode 1), uncomment when use Mode 1
    def call_back(self, msg):

        x, y, xr, t = msg.data

        front_left_power = np.clip(y + x + xr, -0.3, 0.3)
        back_left_power = np.clip(y - x + xr, -0.3, 0.3)
        front_right_power = np.clip(y + x - xr, -0.3, 0.3)
        back_right_power = np.clip(y - x - xr, -0.4, 0.4)

        self.car.recover()
        self.car.left_front_wheel(front_left_power, 100)
        self.car.left_rear_wheel(back_left_power, 100)
        self.car.right_front_wheel(front_right_power, 100)
        self.car.right_rear_wheel(back_right_power, 100)
        """
        v_left = y + x
        v_right = y - x
        # print("v_left= %d, v_right= %d", (v_left, v_right))

        if v_left >= 0 and v_right >= 0:
            self.car.left_drive_forward(min(v_left,1*self.spd_limit), 100)
            self.car.right_drive_forward(min(v_right,1*self.spd_limit), 100)
            #self.car.recover()
        elif v_left <= 0 and v_right <= 0:
            self.car.left_drive_back(max(v_left,-1*self.spd_limit), 100)
            self.car.right_drive_back(max(v_right,-1*self.spd_limit), 100)
            #self.car.recover()
        elif v_left >= 0 and v_right <= 0:
            self.car.left_drive_forward(min(v_left,1*self.spd_limit), 100)
            self.car.right_drive_back(max(v_right,-1*self.spd_limit), 100)
            self.car.recover()
        elif v_left <=0 and v_right >=0:
            self.car.left_drive_back(max(v_left,-1*self.spd_limit), 100)
            self.car.right_drive_forward(min(v_right,1*self.spd_limit), 100)
            self.car.recover()
        else:
            self.car.stop()
        """

        delta_t = (time.time() - t)
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