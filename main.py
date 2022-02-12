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
from round3 import Round3
from round1 import Round1
from round4 import Round4
from round5 import Round5
from round6 import Round6
from round0 import Round0
from master import master_func
from ori import Round2
from os import listdir


dirs = listdir("./..")
if "robojets2" in dirs:
    my_robot = robot2_paramters
elif "robojets1" in dirs:
#else:
    my_robot = robot1_paramters

robo = Robo(my_robot)
run_list = [Round1,Round2,Round3,Round4,Round5,Round6,Round0]
master_func(run_list,robo)
