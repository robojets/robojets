import time
from round_one import Round1

from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import Font
ev3 = EV3Brick()
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile  
from robo import Robo


class ClickType():#מחלקה שמגדירה את סוגי לחיצות של חיישן המגע
    SHORT = 1
    LONG = 2

class user_input():#מחלקה שמגדירה את ממשק המשתמש
    def __init__(self,clicks,click_type):
        self.clicks = clicks
        self.click_type = click_type

def get_user_input(sensr):#פעולה שסופרת את הלחיצות של חיישן המגע
    LONG_PRESS = 1*1000
    TIMEOUT = 0.5*1000
    count = 0
    bul = False
    crnt = time.time()*1000
    while True:
        frst = time.time()*1000
        while sensr.pressed():
            crnt = time.time()*1000
            bul = True
        if bul:
            bul = False
            count +=1
            if (crnt-frst) > LONG_PRESS:
                return  user_input(count, ClickType.LONG)
        if count and ((frst - crnt) >  TIMEOUT):
            return user_input(count, ClickType.SHORT)


def master_func(run_list,robo):#זוהי הפעולה שהופכת את הלחיצות על חיישן המגע להרצות של סבבים
    # Initialize the motors
    global func_list
    func_list = run_list

    crnt_run = 0
    while True:
        ev3.screen.clear()
        big_font = Font(size=24)
        ev3.screen.set_font(big_font)
        ev3.screen.draw_text(50, 60, ("round: " + str(crnt_run+1)+" / " + str(len(run_list))))

        usr_input = get_user_input(robo.touch_sensor)
        if usr_input.click_type == ClickType.SHORT:
            if usr_input.clicks == 1:
                print("execute")
                run_list[crnt_run](robo)               
                crnt_run = (crnt_run+1)%len(run_list)#move next
            elif usr_input.clicks == 2:
                print("forward")
                crnt_run = (crnt_run+1)%len(run_list)#move next
            elif usr_input.clicks ==3 :
                print("back")
                crnt_run = (crnt_run-1)%len(run_list)#move previous
            else:
                print("eror")
        else:
            print("Exit")
            break
