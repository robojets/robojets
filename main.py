#!/usr/bin/env pybricks-micropython
from config import *
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import Font
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile  
from robo import Robo

from round_5 import Round5 as five
from round_4 import Round4 as four
from round_3 import Round3 as three
from round_2 import Round2 as two
from round_1 import Round1 as one
from round0 import Round0 as col
from master import master_func
from os import listdir


dirs = listdir("./..") # we use local file on the brick to identify which Robot we are conneccted to  
if "robojets2" in dirs:
    my_robot = robot2_paramters
elif "robojets1" in dirs:
    my_robot = robot1_paramters
else: 
    my_robot = robot1_paramters # default, in case a file was not defined  

robo = Robo(my_robot)
run_list = [one,two,three,four,five,col]
master_func(run_list,robo)
