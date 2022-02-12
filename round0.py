import time
from round_one import Round1

from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import Font
ev3 = EV3Brick()
#from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
#                                 InfraredSensor, UltrasonicSensor, GyroSensor)
#from pybricks.parameters import Port, Stop, Direction, Button, Color
#from pybricks.tools import wait, StopWatch, DataLog
#from pybricks.robotics import DriveBase
#from pybricks.media.ev3dev import SoundFile, ImageFile  
from robo import Robo
from master import get_user_input,ClickType

def Round0(robo):
     # Initialize the motors.
    crnt_run = "striaght"
    while True:
        ev3.screen.clear()
        big_font = Font(size=24)
        ev3.screen.set_font(big_font)
        ev3.screen.draw_text(50, 40, "straight 1")
        ev3.screen.draw_text(50, 70,"turn 2")
        ev3.screen.draw_text(50, 100,"esc long")

        usr_input = get_user_input(robo.touch_sensor)
        if usr_input.click_type == ClickType.SHORT:
            if usr_input.clicks == 1:
                robo.drive_speed(1000)
                print("straight")
            elif usr_input.clicks == 2:
               robo.turn(360)
               print("circle")
        else:
            print(usr_input.clicks)
            break
