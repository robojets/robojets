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
from ori import Round2 as three
from round1 import Round1 as one
from round5 import Round5 as two
from round0 import Round0 as col
from master import master_func
from ori import Round2
from os import listdir


dirs = listdir("./..") # we use local file on the brick to identify which Robot we are conneccted to  
if "robojets2" in dirs:
    my_robot = robot2_paramters
elif "robojets1" in dirs:
    my_robot = robot1_paramters
else: 
    my_robot = robot1_paramters # default, in case a file was not defined  

robo = Robo(my_robot)
run_list = [one,two,three,col]
master_func(run_list,robo)
