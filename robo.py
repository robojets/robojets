#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor,TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

class arm():
    def __init__(self,mtr_1):
        self.mtr = mtr_1
        self.advance = 0

    def move(self,angle,speed=360):# מזיז את ה זרועה ו עוזב
        self.mtr.run_angle(speed, angle, then=Stop.COAST, wait=True)
        self.advance += angle

    def hold(self,angle,speed=360):  
        self.mtr.run_angle(speed, angle, then=Stop.HOLD, wait=True)
        self.advance += angle

    def reset(self,speed=360):#
        self.mtr.run_angle(speed, self.advance*-1, then=Stop.HOLD, wait=True)

    def up(self,speed=360):
        self.mtr.run_until_stalled(speed, then=Stop.COAST, duty_limit=30)

    def down(self,speed=-360):
        self.mtr.run_until_stalled(speed, then=Stop.COAST, duty_limit=30)


class Robo(DriveBase):
    def __init__(self, my_robot):
        self.touch_sensor = my_robot["touch_sensor"] and TouchSensor(my_robot["touch_sensor"]) or None
        self.color_sensors = [ColorSensor(my_robot["color_sensor_left"]), ColorSensor(my_robot["color_sensor_right"])]
        self.right_wheal = Motor( my_robot["right_wheal"])
        self.left_wheal = Motor( my_robot["left_wheal"])
        self.diameter = my_robot["diameter"]
        self.axl_track = my_robot["axl_track"]
        self.use_gyro = my_robot["use_gyro"]
        

        super().__init__(
            self.left_wheal,
            self.right_wheal,
            self.diameter,
            self.axl_track)

        try:
            self.gyro_sensor = my_robot["gyro_sensor"] and GyroSensor(my_robot["gyro_sensor"]) or None
        except OSError as error: 
            print('gyro sensor is missing!')
        
        try:
            self.lift = my_robot["lift"] and arm(Motor(my_robot["lift"],Direction.CLOCKWISE, my_robot["lift_gears"])) or None
        except OSError as error:
            print('lift motor is missing!')
        
        try:
            self.arm = my_robot["arm"] and arm(Motor(my_robot["arm"],Direction.CLOCKWISE, my_robot["arm_gears"])) or None
        except OSError as error:
            print('arm motor is missing!')
    
    def print_hi(self):
        print("hi")

    def drive_speed(self,distance,speed=250,angle=0):
        self.reset()
        self.drive(speed,angle)
        if distance > 0 :
            while True:
                if self.distance() >= distance:
                    print(self.distance())
                    self.stop()
                    self.reset() 
                    print(self.state())
                    break 
        else:
                while True:
                    if self.distance() > distance:
                        print(self.distance())
                        self.stop()
                        self.reset() 
                        print(self.state())
                        break 
    
    def striaght_speed(self,way,speed):
        self.settings(speed)
        self.straight(way)
    
    def turn_speed(self,angle,speed):
        self.settings(0, 0, speed, 0)
        self.turn(angle)

    def run_until_line(self,num):
        while not self.color_sensors[num].color() == Color.BLACK:
            self.drive(100,0)
        stop()

    def follow_line(self,num):
        line_sensor = self.color_sensors[num]
        
        BLACK = 9
        WHITE = 85
        threshold = (BLACK + WHITE) / 2

        # Set the drive speed at 100 millimeters per second.
        DRIVE_SPEED = 100

        # Set the gain of the proportional line controller. This means that for every
        # percentage point of light deviating from the threshold, we set the turn
        # rate of the drivebase to 1.2 degrees per second.

        # For example, if the light value deviates from the threshold by 10, the robot
        # steers at 10*1.2 = 12 degrees per second.
        PROPORTIONAL_GAIN = 1.2

        # Start following the line endlessly.
        while True:
            # Calculate the deviation from the threshold.
            deviation = line_sensor.reflection() - threshold

            # Calculate the turn rate.
            turn_rate = PROPORTIONAL_GAIN * deviation

            # Set the drive base speed and turn rate.
            self.drive(DRIVE_SPEED, turn_rate)
            # You can wait for a short time or do other things in this loop.

    def drive_stright(self, distance, robotSpeed=250, acceleration=0):
        useGyro = self.use_gyro
        PROPORTIONAL_GAIN = 1.1
        MINIMUM_SPEED = 50
        self.reset()
        if useGyro:
            self.gyro_sensor.reset_angle(0)
        if robotSpeed == 0:
            robotSpeed = 250 # set the speed to its' default value 
        elif robotSpeed < 0: # use distace to set the direction, speed asume positive
            distance = -1 * distance
            robotSpeed = -1 * robotSpeed
        if acceleration < 0: # acceleration must be none negative value 
            acceleration = 0
        currentSpeed = robotSpeed
        speedStep = acceleration / 100 # acceleration is mm/s/s divided to 10ms steps
        angle_correction = 0 # default angle correction if gyro is disabled
        if speedStep > robotSpeed: # maximum acceleration speed step is at maximum speed 
            speedStep = robotSpeed
        elif acceleration > 0: # set the starting speed for acceleration
             currentSpeed = speedStep
    
        if distance < 0: # move backwards
            reverseSpeed = -1 * robotSpeed
            reverseCurrentSpeed = -1 * currentSpeed
            # reverse acceleration drive
            if acceleration > 0:
                while self.distance() > (distance / 2) and reverseCurrentSpeed > reverseSpeed:
                    if useGyro:
                        angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(reverseCurrentSpeed, angle_correction)
                    if reverseCurrentSpeed > reverseSpeed:
                        reverseCurrentSpeed -= speedStep
                    if reverseCurrentSpeed < reverseSpeed:
                        reverseCurrentSpeed = reverseSpeed
                    wait(10)

            acceleration_distance = self.distance()
            # full speed drive
            while (self.distance() + acceleration_distance) > distance:
                    if useGyro:
                        angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(reverseSpeed, angle_correction)
                    wait(10)

            # decceleration drive
            while self.distance() > distance:
                if useGyro:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                self.drive(reverseCurrentSpeed, angle_correction)
                if reverseCurrentSpeed < (-1 * MINIMUM_SPEED) and reverseCurrentSpeed < (-1 * speedStep):
                    reverseCurrentSpeed += speedStep
                else:
                    reverseCurrentSpeed = MINIMUM_SPEED
                wait(10)
        elif distance > 0: # move forwards
            # acceleration drive
            if acceleration > 0:
                while self.distance() < (distance / 2) and currentSpeed < robotSpeed:
                    if useGyro:
                        angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(currentSpeed, angle_correction)
                    if currentSpeed < robotSpeed:
                        currentSpeed += speedStep
                    if currentSpeed > robotSpeed:
                        currentSpeed = robotSpeed
                    wait(10)

            acceleration_distance = self.distance()
            # full speed drive
            while (self.distance() + acceleration_distance) < distance:
                if useGyro:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                self.drive(robotSpeed, angle_correction)
                wait(10)

            # decceleration drive
            while self.distance() < distance:
                if useGyro:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                self.drive(currentSpeed, angle_correction)
                if currentSpeed > MINIMUM_SPEED and currentSpeed > speedStep:
                    currentSpeed -= speedStep
                else:
                    currentSpeed = MINIMUM_SPEED
                wait(10)
        self.stop()
        
    def turn_gyro(self, angle, speed=150, acceleration=0):
        ERROR_TOLERANCE = 4
        MINIMUM_TRUN_SPEED = 80
        wait(10)
        self.reset()
        self.gyro_sensor.reset_angle(0)
        if speed == 0:
            speed = 150 # set speed to its' defaul value
        elif speed < 0: # speed must be positive, we use angle for diraction 
            angle = -1 * angle
            speed = -1 * speed
        turn_angle = angle
        if acceleration < 0: # acceleration must be none negative value
            acceleration = 0
        speedStep = acceleration / 100 # acceleration is mm/s/s it 10 steps/sec rate
        currentSpeed = speed # default speed in case 
        if speedStep > speed: # maximum acceleration speed step is at maximum speed 
            speedStep = speed
        elif acceleration > 0: # set the starting speed for acceleration
             currentSpeed = speedStep
        while abs(turn_angle) >= ERROR_TOLERANCE:
            self.gyro_sensor.reset_angle(0)
            wait(100)
            if turn_angle < 0:
                # acceleration loop
                if acceleration > 0:
                    while (self.gyro_sensor.angle() > (turn_angle / 2)) and (currentSpeed < speed):
                        self.left_wheal.run(speed=(-1 * currentSpeed))
                        self.right_wheal.run(speed=currentSpeed)
                        currentSpeed += speedStep
                        if currentSpeed > speed:
                            currentSpeed = speed
                        wait(10)

                acceleration_angle = self.gyro_sensor.angle()
                # full speed loop
                while (self.gyro_sensor.angle() + acceleration_angle) > turn_angle:
                    self.left_wheal.run(speed=(-1 * speed))
                    self.right_wheal.run(speed=speed)
                    wait(10)
                    
                # decceleration loop
                while self.gyro_sensor.angle() > turn_angle:
                    self.left_wheal.run(speed=(-1 * currentSpeed))
                    self.right_wheal.run(speed=currentSpeed)
                    if currentSpeed > MINIMUM_TRUN_SPEED and (currentSpeed > (2 * speedStep)):
                        currentSpeed -= (2 * speedStep)
                    else:
                        currentSpeed = MINIMUM_TURN_SPEED
                    wait(10)
                    end1 = self.gyro_sensor.angle()
            elif turn_angle > 0:
                # acceleration loop
                if acceleration > 0:
                    while (self.gyro_sensor.angle() < (turn_angle / 2)) and (currentSpeed < speed):
                        self.left_wheal.run(speed=currentSpeed)
                        self.right_wheal.run(speed=(-1 * currentSpeed))
                        currentSpeed += speedStep
                        if currentSpeed > speed:
                            currentSpeed = speed
                        wait(10)
                acceleration_angle = self.gyro_sensor.angle()
                # full speed loop
                while (self.gyro_sensor.angle() + acceleration_angle) < turn_angle:
                    self.left_wheal.run(speed=speed)
                    self.right_wheal.run(speed=(-1 * speed))
                    wait(10)
                
                # decceleration loop
                while self.gyro_sensor.angle() < turn_angle:
                    self.left_wheal.run(speed=currentSpeed)
                    self.right_wheal.run(speed=(-1 * currentSpeed))
                    if currentSpeed > MINIMUM_TRUN_SPEED and (currentSpeed > (2 * speedStep)):
                        currentSpeed -= (2 * speedStep)
                    else:
                        currentSpeed = MINIMUM_TRUN_SPEED
                    wait(10)
            else:
                print("Error: no angle chosen")
            self.left_wheal.brake()
            self.right_wheal.brake()
            wait(100) # stabilize the Gyro
            end_angle = self.gyro_sensor.angle()
            next_turn_angle = turn_angle - end_angle
            if next_turn_angle > ERROR_TOLERANCE:
                next_turn_angle-=ERROR_TOLERANCE
            elif next_turn_angle < -ERROR_TOLERANCE:
                next_turn_angle += ERROR_TOLERANCE
            if abs(next_turn_angle) < abs(turn_angle) and abs(next_turn_angle) >= 3 and abs(next_turn_angle) < 50:
                turn_angle = next_turn_angle
                currentSpeed = speed = MINIMUM_TRUN_SPEED
            else:
                turn_angle = 0 # loop exit condition 
        
        # stop turning
        self.stop()

    def drive2(self, distance, speed=250, angle=0, acceleration=0):
        useGyro = self.use_gyro
        PROPORTIONAL_GAIN = 1.1
        MINIMUM_SPEED = 50
        self.reset()
        if useGyro:
            self.gyro_sensor.reset_angle(0)
        if speed == 0:
            speed = 250 # set the speed to its' default value 
        elif speed < 0: # use distace to set the direction, speed asume positive
            distance = -1 * distance
            speed = -1 * speed
            angle = -1 * angle
        if acceleration < 0: # acceleration must be none negative value 
            acceleration = 0
        currentSpeed = speed
        speedStep = acceleration / 100 # acceleration is mm/s/s divided to 10ms steps
        angle_correction = angle # default angle correction if gyro is disabled
        if speedStep > speed: # maximum acceleration speed step is at maximum speed 
            speedStep = speed
        elif acceleration > 0: # set the starting speed for acceleration
             currentSpeed = speedStep
    
        if distance < 0: # move backwards
            reverseSpeed = -1 * speed
            reverseCurrentSpeed = -1 * currentSpeed
            # reverse acceleration drive
            if acceleration > 0:
                while self.distance() > (distance / 2) and reverseCurrentSpeed > reverseSpeed:
                    if useGyro:
                        angle_correction = angle - PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(reverseCurrentSpeed, angle_correction)
                    if reverseCurrentSpeed > reverseSpeed:
                        reverseCurrentSpeed -= speedStep
                    if reverseCurrentSpeed < reverseSpeed:
                        reverseCurrentSpeed = reverseSpeed
                    wait(10)

            acceleration_distance = self.distance()
            # full speed drive
            while (self.distance() + acceleration_distance) > distance:
                    if useGyro:
                        angle_correction = -angle - PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(reverseSpeed, angle_correction)
                    wait(10)

            # decceleration drive
            while self.distance() > distance:
                if useGyro:
                    angle_correction = angle - PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                self.drive(reverseCurrentSpeed, angle_correction)
                if reverseCurrentSpeed < (-1 * MINIMUM_SPEED) and reverseCurrentSpeed < (-1 * speedStep):
                    reverseCurrentSpeed += speedStep
                else:
                    reverseCurrentSpeed = MINIMUM_SPEED
                wait(10)
        elif distance > 0: # move forwards
            # acceleration drive
            if acceleration > 0:
                while self.distance() < (distance / 2) and currentSpeed < speed:
                    if useGyro:
                        angle_correction = angle - PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(currentSpeed, angle_correction)
                    if currentSpeed < speed:
                        currentSpeed += speedStep
                    if currentSpeed > speed:
                        currentSpeed = speed
                    wait(10)

            acceleration_distance = self.distance()
            # full speed drive
            while (self.distance() + acceleration_distance) < distance:
                if useGyro:
                    angle_correction = angle - PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                self.drive(speed, angle_correction)
                wait(10)

            # decceleration drive
            while self.distance() < distance:
                if useGyro:
                    angle_correction = angle - PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                self.drive(currentSpeed, angle_correction)
                if currentSpeed > MINIMUM_SPEED and currentSpeed > speedStep:
                    currentSpeed -= speedStep
                else:
                    currentSpeed = MINIMUM_SPEED
                wait(10)
        self.stop()
    
    #def turn_radius(self, angle, radius)

