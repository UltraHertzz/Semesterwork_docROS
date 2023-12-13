"""
  ROBOTCAR control via xbox controller
  *************************************
  (if left red light of xbox controller is on, it is properly connected to the raspberry)

  * drive car with left joystick
        - drive_forward
        - drive_left
        - drive_right
        - drive_back
        - stop_car

  * stop script with 'X'-button

  Last update: 09/2020
"""


"""
record the initial pin connection on car with RaspberryPi 3

Physical    BCM     Device      color
1           -       IMU         red
--- Begin of Bundle ---
3           2       FL          white
5           3       FL          brown
7           4       FL          light purple
11          17      RL          black
13          27      RL          red with white stripe
15          22      RL          light orange with white stripe
---  END of Bundle  ---
19          10      IMU         black
--- Begin of Bundle ---
16          23      FR          orange
18          24      FR          yellow
22          25      FR          green
24          8       RR          red
26          7       RR          yellow
36          16      RR          orange
---  END of Bundle  ---
20          0V      car_con     black

FL: front left wheel
FR: front right wheel
"""

import OPi.GPIO as GPIO
import xbox
import time

# enable GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
    

# initializing GPIO
# correct Rear Right
GPIO.setup(24, GPIO.OUT)       # GPIO.setup(8, GPIO.OUT) # enable D 7-8 1        8
GPIO.setup(26, GPIO.OUT)       # GPIO.setup(7, GPIO.OUT) # IN8                   7
GPIO.setup(36, GPIO.OUT)       # GPIO.setup(16, GPIO.OUT) # IN7 should be False  16
# correct Front Right
GPIO.setup(16, GPIO.OUT)       # IN6 False -> Forward       23
GPIO.setup(18, GPIO.OUT)       # IN5                        24
GPIO.setup(22, GPIO.OUT)       # enable C 5-6 Forward Right 25
# correct Front Left
GPIO.setup( 3, GPIO.OUT)       # IN2                        2
GPIO.setup( 5, GPIO.OUT)       # enable A 1-2               3
GPIO.setup( 7, GPIO.OUT)       # IN1 False -> Forward       4
# correct Rear Left
GPIO.setup(11, GPIO.OUT)       # IN3 False -> Forward       17
GPIO.setup(13, GPIO.OUT)       # IN4                        27
GPIO.setup(15, GPIO.OUT)       # enable B 3-4               22

# enable all motors
GPIO.output(15, True)          # 22
GPIO.output(24, True)          # 8
GPIO.output(22, True)          # 25
GPIO.output(5 , True)          # 3

# make motor forward
GPIO.output(36, False)         # 16
GPIO.output(16, False)         # 23
GPIO.output( 7, False)         # 4
GPIO.output(11, False)         # 17


# enable speed control
pwm_fl = GPIO.PWM(chip=5, pin= 16, frequency=80, duty_cycle_percent=0) # 2
pwm_fr = GPIO.PWM(chip=7, pin= 7, frequency=80, duty_cycle_percent=0) # 24
pwm_bl = GPIO.PWM(chip=2, pin=13, frequency=80, duty_cycle_percent=0) # 27
pwm_br = GPIO.PWM(chip=2, pin=26, frequency=80, duty_cycle_percent=0) # 7

# pwm_fl.start_pwm()
pwm_fr.start_pwm()
pwm_bl.start_pwm()
pwm_br.start_pwm()


def drive_forward(joy_left_x, joy_left_y, init_speed):
    GPIO.output(36, False)
    GPIO.output(16, False)
    GPIO.output( 7, False)
    GPIO.output(11, False)

    GPIO.output(26, True)
    GPIO.output(18, True)
    GPIO.output( 3, True)
    GPIO.output(13, True)
    # front wheels are driven

    """
    pwm_fl.start_pwm()
    pwm_fr.start_pwm()
    pwm_bl.start_pwm()
    pwm_br.start_pwm()

    value = init_speed * joy_left_y * 60
    pwm_fl.duty_cycle(value)            # input range (0, 40)
    pwm_fr.duty_cycle(value)            # input range (0, 40)
    pwm_bl.duty_cycle(value)            # input range (0, 40)
    pwm_br.duty_cycle(value)            # input range (0, 40)
    """
    time.sleep(0.02)
    print("Forward")


def stop_car():
    # all wheels are stopped
    """
    pwm_fl.stop_pwm()
    pwm_fr.stop_pwm()
    pwm_bl.stop_pwm()
    pwm_br.stop_pwm()
    """
    GPIO.output(15, False)         # 22
    GPIO.output(24, False)         # 8
    GPIO.output(22, False)         # 25
    GPIO.output(5 , False)             # 3
    print("STOP")

def drive_right(joy_left_x, joy_left_y, init_speed):
#   GPIO.output(23,True)
    GPIO.output(7,False)
#   GPIO.output(16,True)
    GPIO.output(11,False)
    # left wheels are driven, right wheels are blocked
    """
    pwm_fl.start()
    pwm_bl.start()
#   pwm_br.start()
#   pwm_fr.start()

    value = init_speed * joy_left_x * 60
    pwm_fl.ChangeDutyCycle(value)           # input range (0, 40)
    pwm_bl.ChangeDutyCycle(value)           # input range (0, 40)
#   pwm_fr.ChangeDutyCycle(2 * value)
#   pwm_br.ChangeDutyCycle(2 * value)
    """
    time.sleep(0.02)
    print("Right")
def drive_left(joy_left_x, joy_left_y, init_speed):
    GPIO.output(36,False)
    # GPIO.output(4,True)
    GPIO.output(16,False)
 #       GPIO.output(17,True)
    """
    # right wheels are driven, left wheels are blocked
    pwm_fr.start_pwm()
    pwm_br.start_pwm()
#   pwm_fl.start_pwm()
#   pwm_bl.start_pwm()

    value = init_speed * (-1*joy_left_x) * 60
    pwm_fr.duty_cycle(value)            # input range (0, 40)
    pwm_br.duty_cycle(value)            # input range (0, 40)
#   pwm_fl.duty_cycle(2 * value)
#   pwm_bl.duty_cycle(2 * value)
    """
    time.sleep(0.02)
    print("Left")


def drive_back(joy_left_x, joy_left_y, init_speed):
    # front wheels are driven
    # with opposite running direction

    GPIO.output(36, True)
    GPIO.output(16, True)
    GPIO.output( 7, True)
    GPIO.output(11, True)

    GPIO.output(26, False)
    GPIO.output(18, False)
    GPIO.output( 3, False)
    GPIO.output(13, False)
    """
    pwm_fl.start_pwm()
    pwm_fr.start_pwm()
    pwm_bl.start_pwm()
    pwm_br.start_pwm()

    value = init_speed * (100 + 40*joy_left_y)
    pwm_fl.duty_cycle(value)            # input range (60, 100)
    pwm_fr.duty_cycle(value)            # input range (60, 100)
    """
    time.sleep(0.02)
    print("Go Back")

if __name__ == '__main__':
    # initialize joystick
    joy = xbox.Joystick()
    stop_flag = False
    # while 'X'-button is not pressed
    while not joy.X():
        if joy.B():
            stop_flag = True
        while stop_flag:
            print("car stopped!")
            stop_car()
            if joy.A():
                print("restart! take care!")
                GPIO.output(15, True)          # 22
                GPIO.output(24, True)          # 8
                GPIO.output(22, True)          # 25
                GPIO.output(5 , True)          # 3
                stop_flag = False
        x, y = joy.leftStick()
        # front wheels are running forward by default
        GPIO.output(16, False)
        GPIO.output( 7, False)

        #if (y == 0 and x == 0):
            #stop_car()

        if y >= 0:
            if x > 0:
                drive_right(x, y, 1)
            elif x < 0:
                drive_left(x, y, 1)
            else:
                drive_forward(x, y, 1)

        else:
            drive_back(x, y, 1)

        print(x, y)

# cleanup and close
joy.close()
GPIO.cleanup()

