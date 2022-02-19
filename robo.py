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

    #####################################################################
    # Function name: move(angle, speed)
    # Description: moves the robot arm to a given angle and a given speed
    #               relesing the engine power when done
    # Parameters:
    #   angle - how many degree to turn the arm angine, positive or negative 
    #   speed - what speed to turn the arm engine
    def move(self, angle, speed=360):# מזיז את ה זרועה ו עוזב
        self.mtr.run_angle(speed, angle, then=Stop.COAST, wait=True)
        self.advance += angle
        return

    #####################################################################
    # Function name: hold(angle, speed)
    # Description: moves the robot arm to a given angle and a given speed
    #               keeping the engine power when done to hold an object
    # Parameters:
    #   angle - how many degree to turn the arm angine, positive or negative 
    #   speed - what speed to turn the arm engine
    def hold(self, angle, speed=360):  
        self.mtr.run_angle(speed, angle, then=Stop.HOLD, wait=True)
        self.advance += angle
        return

    #####################################################################
    # Function name: reset(speed)
    # Description: moves the robot arm to the starting position
    # Parameters:
    #   speed - what speed to turn the arm engine
    def reset(self, speed=360):#
        self.mtr.run_angle(speed, self.advance*-1, then=Stop.HOLD, wait=True)
        self.advance = 0
        return

    #####################################################################
    # Function name: up(speed)
    # Description: moves the robot arm all the way up and releasing the hold
    # Parameters:
    #   speed - what speed to turn the arm engine
    def up(self, speed=360):
        self.mtr.run_until_stalled(speed, then=Stop.COAST, duty_limit=30)
        self.advance += self.angle()
        return

    #####################################################################
    # Function name: down(speed)
    # Description: moves the robot arm all the way down and releasing the hold
    # Parameters:
    #   speed - what speed to turn the arm engine
    def down(self, speed=-360):
        self.mtr.run_until_stalled(speed, then=Stop.COAST, duty_limit=30)
        self.advance += self.angle()
        return


class Robo(DriveBase):
    def __init__(self, my_robot):
        self.touch_sensor = my_robot["touch_sensor"] and TouchSensor(my_robot["touch_sensor"]) or None
        self.color_sensors = [ColorSensor(my_robot["color_sensor_left"]), ColorSensor(my_robot["color_sensor_right"])]
        self.right_wheal = Motor( my_robot["right_wheal"])
        self.left_wheal = Motor( my_robot["left_wheal"])
        self.diameter = my_robot["diameter"]
        self.axl_track = my_robot["axl_track"]
        self.use_gyro = my_robot["use_gyro"]
        # intialize the DriveBase class
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
        return

    #####################################################################
    # Function name: drive_speed(distance, speed, angle)
    # Description: drive the robot back or forth in given speed and angle
    # Parameters:
    #   distance - driving distance, can be positive or negative
    #   speed    - driving speed, can be positive or negative
    #   angle    - driving angle, can be positive or negative
    def drive_speed(self, distance, speed=250, angle=0):
        if speed==0 or distance == 0:
            return # nothing to do
    
        # use speed to indicate the driving direciton, distance is always positive
        if distance < 0 and speed > 0:
            speed = speed * -1
        elif distance > 0 and speed < 0
            distance = distance * -1

        # start driving
        self.reset()
        self.drive(speed, angle)
        if distance > 0 :
            while self.distance() < distance:
                wait(10)
        else: # distance <= 0
            while self.distance() > distance:
                wait(10)
        # stop he motors before returning to the caller
        self.stop()
        return

    #####################################################################
    # Function name: straight_speed(distance, speed)
    # Description: drives the robot straight forward or backward at a given speed
    # Parameters:
    #   distance - how much to drive
    #   speed    - how fast to drive
    def straight_speed(self, distance, speed):
        self.reset()
        self.settings(speed)
        self.straight(distance)
        # stop he motors before returning to the caller
        self.stop()
        return
    
    #####################################################################
    # Function name: turn_speed(angle, speed)
    # Description: turn the robot left or right at a given speed
    # Parameters:
    #   angle - how much to turn
    #   speed - how fast to turn
    def turn_speed(self, angle, speed):
        self.reset()
        self.settings(0, 0, speed, 0)
        self.turn(angle)
        # stop he motors before returning to the caller
        self.stop()
        return

    #####################################################################
    # Function name: drive_until_line(sensor, color)
    # Description: drive the robot untill one of the light sensor hit a black or white line 
    # Parameters: 
    #   sensor - which sensor to use 0=right, 1=left, -1=any
    #   color  - which color to stop 0=black, 1=white
    # Return value: return the index of the sensor that stop the robot, or -1 if fail to detect a line
    def drive_until_line(self, sensor=-1, color=0):
        DETECTION_THRESHOLD = 10 # number of hits before we stop
        BLACK = 9  # define that black is reflection of 0-9 
        WHITE = 85 # define that white is reflection of 85-100
        DRIVE_SPEED = 100 # deriving speed towords the line
        LOOP_DELAY = 20   # constant delay between color sensor samples 

        if sensor < -1 or sensor > 1:
            raise Exception("drive_until_line input error: invalid sensor index")
        if color<0 or color>1:
            raise Exception("drive_until_line input error: invalid solor number")
        
        stopSensorIndex = (sensor+2)%2
        otherSensorIndex = (stopSensor+1)%2
        stopSensor  = self.color_sensors[stopSensorIndex]
        otherSensor = self.color_sensors[otherSensorIndex]
        
        # count the number of consecutive line detections on each sensor before stoping the robot 
        countStopSensor  = 0
        countOtherSensor = 0

        while True:
            self.drive(DRIVE_SPEED,0)
            wait(LOOP_DELAY) # constant loop delay
            
            stopSensorValue  = stopSensor.reflection()
            otherSensorValue = otherSensor.reflection()

            if ((stopSensorValue <= BLACK) and color == 0) or ((stopSensorValue >= WHITE) and color == 1):
                countStopSensor += 1
            else:
                countStopSensor = 0

            if ((otherSensorValue <= BLACK) and color == 0) or ((otherSensorValue >= WHITE) and color == 1):
                countOtherSensor += 1
            else:
                countOtherSensor = 0
            if (countStopSensor>=DETECTION_THRESHOLD) or (sensor==-1 and (countOtherSensor>=DETECTION_THRESHOLD)):
                break
        # stop he motors before returning to the caller
        self.stop()
        if stopSensorValue>=DETECTION_THRESHOLD:
            return stopSensorIndex
        else:
            return otherSensorIndex

    #####################################################################
    # Function name: follow_line(sensor, distance, stopOnBlackLine)
    # Description: follow black line on the robot game map using color sensor 
    #               and proportional feedback loop
    # Parameters: 
    #   sensor          - which sensor index is useed to follow the line 0=right, 1=left, -1=auto-select
    #   distance        - if distance>0, the maximum distance to drive, or no-imit
    #   stopOnBlackLine - True=stops the robot if the other sensor hit black color
    def follow_line(self, sensor=-1, distance=0, stopOnBlackLine=True):
        # number of black line hits on second sensor before stopping the robot
        STOP_ON_BLACK_THRESHOLD = 4 
        # Set the drive speed at 100 millimeters per second.
        DRIVE_SPEED = 100
        # Set the gain of the proportional line controller. This means that for every
        # percentage point of light deviating from the threshold, we set the turn
        # rate of the drivebase to 1.2 degrees per second.
        # For example, if the light value deviates from the threshold by 10, the robot
        # steers at 10*1.2 = 12 degrees per second.
        PROPORTIONAL_GAIN = 1.2
        # define that black is reflection of 0-9 
        BLACK = 9
        # define that white is reflection of 85-100 
        WHITE = 85
        # defines the reflected color value to follow as average between black and white 
        threshold = (BLACK + WHITE) / 2 

        if sensor < 0 or sensor > -1:
            raise Exception("follow_line input error: invalid sensor index")
        if distance <= 0:
            distance = 1000000 # arbitary large number
        if stopOnBlackLine != True:
            stopOnBlackLine = False
        
        # if self detection is selected, drive until any sensor hits a white line
        if sensor == -1:
            # drive untill the robot hits a white line with one of it's sensors
            sensor = drive_until_line(sensor, 1)
        
        lineSensor = self.color_sensors[sensor]
        stopSensor = self.color_sensors[(sensor+1)%2]

        speed = 100
        direction = -1
        maxCount = 1
        while direction == -1:
            count = 0
            # detect if the robot is at the right side of the black line
            while (direction == -1) and (count < maxCount):
                self.left_wheal.run(speed=(-1 * speed))
                self.right_wheal.run(speed=speed)
                wait(10)
                count += 1
                if line_sensor.reflection() <= BLACK:
                    direction = 0 # right to the black line

            count = 0
            # detect if the robot is at the left side of the black line
            while (direction == -1) and (count < (maxCount*2)):
                self.left_wheal.run(speed=speed)
                self.right_wheal.run(speed=(-1 * speed))
                wait(10)
                count += 1
                if line_sensor.reflection() <= BLACK:
                    direction = 1 # left to the black line
        
        stopCount = 0
        # sero the distance count
        self.reset()
        # Start following the line
        while True:
            # Calculate the deviation from the threshold.
            deviation = line_sensor.reflection() - threshold

            # Calculate the turn rate.
            turn_rate = PROPORTIONAL_GAIN * deviation
            if direction == 0: # right to the black line
                turn_rate = turn_rate * -1

            # Set the drive base speed and turn rate.
            self.drive(DRIVE_SPEED, turn_rate)
            wait(20)
            # You can wait for a short time or do other things in this loop.
            if stopOnBlackLine:
                if stopSensor.reflection() <= BLACK:
                    stopCount += 1
                else:
                    stopCount = 0
                if stopCount == STOP_ON_BLACK_THRESHOLD:
                    break
        # stop he motors before returning to the caller
        self.stop()
        return

    #####################################################################
    # Function name: drive_straight(distance, speed, acceleration)
    # Description: drives the Robot in stright line in a given speed and acceleration
    # Parameters:
    #   distance     - driving distance
    #   speed        - driving speed
    #   acceleration - acceleration rate in mm/s/s
    def drive_straight(self, distance, speed=250, acceleration=0):
        useGyro = self.use_gyro
        PROPORTIONAL_GAIN = 1.1
        MINIMUM_SPEED = 50
        # sero the distance count
        self.reset()
        if useGyro:
            self.gyro_sensor.reset_angle(0)
        if speed == 0:
            speed = 250 # set the speed to its' default value 
        elif speed < 0: # use distace to set the direction, speed asume positive
            distance = -1 * distance
            speed = -1 * speed
        if acceleration < 0: # acceleration must be none negative value 
            acceleration = 0
        currentSpeed = speed
        speedStep = acceleration / 100 # acceleration is mm/s/s divided to 10ms steps
        angle_correction = 0 # default angle correction if gyro is disabled
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
                        angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(reverseCurrentSpeed, angle_correction)
                    wait(10)
                    if reverseCurrentSpeed > reverseSpeed:
                        reverseCurrentSpeed -= speedStep
                    if reverseCurrentSpeed < reverseSpeed:
                        reverseCurrentSpeed = reverseSpeed

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
                wait(10)
                if reverseCurrentSpeed < (-1 * MINIMUM_SPEED) and reverseCurrentSpeed < (-1 * speedStep):
                    reverseCurrentSpeed += speedStep
                else:
                    reverseCurrentSpeed = MINIMUM_SPEED
        elif distance > 0: # move forwards
            # acceleration drive
            if acceleration > 0:
                while self.distance() < (distance / 2) and currentSpeed < speed:
                    if useGyro:
                        angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(currentSpeed, angle_correction)
                    wait(10)
                    if currentSpeed < speed:
                        currentSpeed += speedStep
                    if currentSpeed > speed:
                        currentSpeed = speed

            acceleration_distance = self.distance()
            # full speed drive
            while (self.distance() + acceleration_distance) < distance:
                if useGyro:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                self.drive(speed, angle_correction)
                wait(10)

            # decceleration drive
            while self.distance() < distance:
                if useGyro:
                    angle_correction = -1 * PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                self.drive(currentSpeed, angle_correction)
                wait(10)
                if currentSpeed > MINIMUM_SPEED and currentSpeed > speedStep:
                    currentSpeed -= speedStep
                else:
                    currentSpeed = MINIMUM_SPEED
        # stop he motors before returning to the caller
        self.stop()
        return
    
    #####################################################################
    # Function name: turn_gyro(angle, speed, acceleration)
    # Description: turn the robot to a given angle using the gyrosensor 
    #              if suported  at a given speed and acceleration
    # Parameters: 
    #   angle        - the angle to trun
    #   speed        - the speed of the turn
    #   acceleration - acceleration rate in mm/s/s
    def turn_gyro(self, angle, speed=150, acceleration=0):
        ERROR_TOLERANCE = 4
        MINIMUM_TRUN_SPEED = 80
        if speed == 0:
            speed = 150 # set speed to its' defaul value
        elif speed < 0: # speed must be positive, we use angle for diraction 
            angle = -1 * angle
            speed = -1 * speed
        turn_angle = angle

        # don't try to turn using gyro if the gyro is off or the angle is below threshold
        if (self.use_gyro == False) or (abs(turn_angle) >= ERROR_TOLERANCE):
            self.turn(angle)
            self.stop()
            return

        # acceleration must be none negative value
        if acceleration < 0:
            acceleration = 0
        # acceleration is mm/s/s it 10 steps/sec rate
        speedStep = acceleration / 100
        currentSpeed = speed # default speed in case 
        # maximum acceleration speed step is at maximum speed 
        if speedStep > speed:
            speedStep = speed
        elif acceleration > 0: # set the starting speed for acceleration
             currentSpeed = speedStep
        # reset the gyro sensors before starting to turn 
        self.gyro_sensor.reset_angle(0)
        # turn the robot to the left
        if turn_angle < 0:
            # acceleration loop
            if acceleration > 0:
                while (self.gyro_sensor.angle() > (turn_angle / 2)) and (currentSpeed < speed):
                    self.left_wheal.run(speed=(-1 * currentSpeed))
                    self.right_wheal.run(speed=currentSpeed)
                    wait(10)
                    currentSpeed += speedStep
                    if currentSpeed > speed:
                        currentSpeed = speed

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
                wait(10)
                if currentSpeed > MINIMUM_TRUN_SPEED and (currentSpeed > (2 * speedStep)):
                    currentSpeed -= (2 * speedStep)
                else:
                    currentSpeed = MINIMUM_TURN_SPEED
        # turn the robot to the right
        elif turn_angle > 0:
            # acceleration loop
            if acceleration > 0:
                while (self.gyro_sensor.angle() < (turn_angle / 2)) and (currentSpeed < speed):
                    self.left_wheal.run(speed=currentSpeed)
                    self.right_wheal.run(speed=(-1 * currentSpeed))
                    wait(10)
                    currentSpeed += speedStep
                    if currentSpeed > speed:
                        currentSpeed = speed

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
                wait(10)
                if currentSpeed > MINIMUM_TRUN_SPEED and (currentSpeed > (2 * speedStep)):
                    currentSpeed -= (2 * speedStep)
                else:
                    currentSpeed = MINIMUM_TRUN_SPEED

        # stop the robbot from spinning 
        self.left_wheal.brake()
        self.right_wheal.brake()
        wait(100) # stabilize the Gyro
        end_angle = self.gyro_sensor.angle()
        correction_angle =turn_angle - end_angle
        if abs(correction_angle) > ERROR_TOLERANCE:
            self.turn(correction_angle)
        
        # stop he motors before returning to the caller
        self.stop()
        return

    #####################################################################
    # Function name: drive2(distance, speed, angle, acceleration)
    # Description: it implements driving the Robot stright or in angle 
    #              with or without gyro assistant, with or without acceleration.
    # Parameters:
    #   distance     - the distance to drive, can be positive or negative
    #   speed        - the driving speed
    #   angle        - driving angle, 0=sraight 
    #   acceleration - acceleration rate in mm/s/s
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
                    wait(10)
                    if reverseCurrentSpeed > reverseSpeed:
                        reverseCurrentSpeed -= speedStep
                    if reverseCurrentSpeed < reverseSpeed:
                        reverseCurrentSpeed = reverseSpeed

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
                wait(10)
                if reverseCurrentSpeed < (-1 * MINIMUM_SPEED) and reverseCurrentSpeed < (-1 * speedStep):
                    reverseCurrentSpeed += speedStep
                else:
                    reverseCurrentSpeed = MINIMUM_SPEED
        elif distance > 0: # move forwards
            # acceleration drive
            if acceleration > 0:
                while self.distance() < (distance / 2) and currentSpeed < speed:
                    if useGyro:
                        angle_correction = angle - PROPORTIONAL_GAIN * self.gyro_sensor.angle()
                    self.drive(currentSpeed, angle_correction)
                    wait(10)
                    if currentSpeed < speed:
                        currentSpeed += speedStep
                    if currentSpeed > speed:
                        currentSpeed = speed

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
                wait(10)
                if currentSpeed > MINIMUM_SPEED and currentSpeed > speedStep:
                    currentSpeed -= speedStep
                else:
                    currentSpeed = MINIMUM_SPEED

        # stop he motors before returning to the caller
        self.stop()
        return
    

