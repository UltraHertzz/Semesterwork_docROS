from car_driver_localpkg.OPi import GPIO as GPIO


class CarController_Base():

    def __init__(self) -> None:
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # calibrate Right Rear Wheel
            # 18 -> pwm pin (yellow)
            # 19 -> gpio pin  (orange)
            # 22 -> enable (red)
        GPIO.setup([18, 19, 22], GPIO.OUT) 

        # calibrate Right Front Wheel
            # 31 -> gpio pin (orange)
            # 33 -> pwm pin (yellow)
            # 35 -> enable (green)
        GPIO.setup([31, 33, 35], GPIO.OUT) 

        # calibrate Left Rear Wheel
            # 13 -> gpio pin (black)
            # 15 -> enbale (light brown)
            # 16 -> pwm pin (red)
        GPIO.setup([13, 15, 16], GPIO.OUT)

        # calibrate Left Front Wheel
            # 10 -> enable (brown)
            # 11 -> gpio pin
            # 12 -> pwm pin
        GPIO.setup([10, 11, 12], GPIO.OUT)

        # set PWM pin
        self.pwm_rr = GPIO.PWM(7,0,50,100) # pwmchip7 -> GPIO pin 33(febf0020) right rear
        self.pwm_rf = GPIO.PWM(6,0,50,100) # pwmchip6 -> GPIO pin 18(fd8b0010) right front
        self.pwm_lr = GPIO.PWM(5,0,50,100) # pwmchip5 -> GPIO pin 16(febf0000) left rear
        self.pwm_lf = GPIO.PWM(4,0,50,100) # pwmchip4 -> GPIO pin 12(febe0030) left front 

        # Enable all wheels
        self.enable_pin_list = [35, 22, 15, 10]
        GPIO.output(self.enable_pin_list, True)

        self.pwm_rr.start_pwm()
        self.pwm_rf.start_pwm()
        self.pwm_lr.start_pwm()
        self.pwm_lf.start_pwm()

        self.stop_flag = False

        # in practice the rear right wheel will be slow, change this to balance the wheel spin 
        # (notice that the interval of pwm input is [0,100])
        self.compensate_rate_rr = 1.0 

        self.compensate_rate_rf = 1.0
        self.compensate_rate_lr = 1.0
        self.compensate_rate_lf = 1.0

    def stop(self):

        GPIO.output(self.enable_pin_list, False)

    def recover(self):

        GPIO.output(self.enable_pin_list, True)

    def exit(self):

        self.pwm_rr.stop_pwm()
        self.pwm_rf.stop_pwm()
        self.pwm_lr.stop_pwm()
        self.pwm_lf.stop_pwm()

        self.pwm_rr.pwm_close()
        self.pwm_rf.pwm_close()
        self.pwm_lr.pwm_close()
        self.pwm_lf.pwm_close()

        GPIO.cleanup()


class CarController_SideTurn(CarController_Base):

    """
    In this class, the car will turn base on speed difference between two sides of the wheel.
    """

    def __init__(self) -> None:
        
        super().__init__()
        self.wheel_separation = 0.14

    def left_drive_forward(self, desired_speed, init_speed):
    
        GPIO.output([13,11],False)

        value_l = 100 - init_speed*desired_speed

        self.pwm_lr.duty_cycle(self.compensate_rate_lr*value_l)
        self.pwm_lf.duty_cycle(self.compensate_rate_lf*value_l)

        # time.sleep(0.02) 
        #print("Forward")

    def left_drive_back(self, desired_speed,init_speed):

        GPIO.output([13,16,
                     11,12], True)
        

        value_l = -1*init_speed*desired_speed

        self.pwm_lr.duty_cycle(self.compensate_rate_lr*value_l)
        self.pwm_lf.duty_cycle(self.compensate_rate_lf*value_l)

        #time.sleep(0.02) 
        #print("Backward")
    def right_drive_forward(self, desired_speed, init_speed):
    
        GPIO.output([31,19],False)

        value_r = 100 - init_speed*desired_speed

        self.pwm_rr.duty_cycle(self.compensate_rate_rr*value_r)			
        self.pwm_rf.duty_cycle(self.compensate_rate_rf*value_r)

        # time.sleep(0.02) 
        #print("Forward")

    def right_drive_back(self, desired_speed ,init_speed):

        GPIO.output([31,33, 
                     18,19], True)
        
        value_r = -1*init_speed*desired_speed

        self.pwm_rr.duty_cycle(self.compensate_rate_rr*value_r)		
        self.pwm_rf.duty_cycle(self.compensate_rate_rf*value_r)



class CarController_CenterTurn(CarController_Base):

    """
    In this class, the car will turn around along its center axis.
    """

    def __init__(self) -> None:

        super().__init__()

    def drive_forward(self, joy_left_y, init_speed):
    
        GPIO.output([31,19,13,11],False)

        value_rr = 100 - init_speed*self.compensate_rate_rr*joy_left_y
        value_rf = 100 - init_speed*self.compensate_rate_rf*joy_left_y
        value_lr = 100 - init_speed*self.compensate_rate_lr*joy_left_y
        value_lf = 100 - init_speed*self.compensate_rate_lf*joy_left_y


        self.pwm_rr.duty_cycle(value_rr)			
        self.pwm_rf.duty_cycle(value_rf)
        self.pwm_lr.duty_cycle(value_lr)
        self.pwm_lf.duty_cycle(value_lf)

        # time.sleep(0.02) 
        # print("Forward")

    def drive_back(self, joy_left_y, init_speed):

        GPIO.output([31,33, 
                     18,19, 
                     13,16,
                     11,12], True)

        value_rr = -1*init_speed*joy_left_y*self.compensate_rate_rr
        value_rf = -1*init_speed*joy_left_y*self.compensate_rate_rf
        value_lr = -1*init_speed*joy_left_y*self.compensate_rate_lr
        value_lf = -1*init_speed*joy_left_y*self.compensate_rate_lf


        self.pwm_rr.duty_cycle(value_rr)		
        self.pwm_rf.duty_cycle(value_rf)
        self.pwm_lr.duty_cycle(value_lr)
        self.pwm_lf.duty_cycle(value_lf)

        #time.sleep(0.02) 
        # print("Backward")

    def drive_left(self, joy_left_x, init_speed):
        # x < 0, left side backward, both pin set to true; right side forward, both pin set to false

        #GPIO.output([31,19,13,16,11,12],[False, False, True, True, True, True])
        GPIO.output([31,33,18,19],False)
        GPIO.output([13,16,11,12],True)

        value_rr = 100 + joy_left_x*init_speed*self.compensate_rate_rr
        value_rf = 100 + joy_left_x*init_speed*self.compensate_rate_rf
        value_lr = -1*joy_left_x*init_speed*self.compensate_rate_lr
        value_lf = -1*joy_left_x*init_speed*self.compensate_rate_lf


        self.pwm_rr.duty_cycle(value_rr)		
        self.pwm_rf.duty_cycle(value_rf)
        self.pwm_lr.duty_cycle(value_lr)
        self.pwm_lf.duty_cycle(value_lf)

    def drive_right(self, joy_left_x, init_speed):
        # x > 0

        #GPIO.output([31,33,18,19,13,11],[True, True, True, True, False, False])
        GPIO.output([31,33,18,19],True)
        GPIO.output([11,12,13,16],False)

        value_rr = joy_left_x*init_speed*self.compensate_rate_rr
        value_rf = joy_left_x*init_speed*self.compensate_rate_rf
        value_lr = 100 - joy_left_x*init_speed*self.compensate_rate_lr
        value_lf = 100 - joy_left_x*init_speed*self.compensate_rate_lf
        

        self.pwm_rr.duty_cycle(value_rr)		
        self.pwm_rf.duty_cycle(value_rf)
        self.pwm_lr.duty_cycle(value_lr)
        self.pwm_lf.duty_cycle(value_lf)





