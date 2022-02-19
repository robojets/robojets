from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import Font
ev3 = EV3Brick()
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile  
from robo import Robo

#

robot1_paramters = {
    #(פרמטרים של רובוט (זה תואם לפרמטרים של רובוט שתיים
    "diameter" : 70.8,#אחראי על קוטר גלגל
    "axl_track":96,#אחראי על ציר סיבוב
    "left_wheal" : Port.B,#גלגל שמאלי 
    "right_wheal" : Port.A,#גלגל ימני
    "lift":Port.C,#מנוע זרוע מעלית
    "arm":Port.D,#מנוע זרוע צידי
    "lift_gears":[16,40,24],#גלגלי שיניים של כל זרוע
    "arm_gears":[24,24,40],#גלגלי שיניים של כל זרוע
    "color_sensor_left": Port.S3,#סנסור-צבע-1
    "color_sensor_right": Port.S2,#סנסור-צבע-2
    "touch_sensor": Port.S1,#סנסור-מגע
    "gyro_sensor": Port.S4,#סנסור-גיירו
    "use_gyro": False
}

robot2_paramters = {
    "diameter" : 73.5,
    "axl_track": 110,
    "left_wheal" :Port.A,
    "right_wheal" : Port.C,
    "lift":Port.B,
    "arm":Port.D,
    "lift_gears":[16,40,24],
    "arm_gears":[8,24,40],
    "color_sensor_left": Port.S1,
    "color_sensor_right": Port.S4,
    "touch_sensor": Port.S3,
    "gyro_sensor": Port.S2,
     "use_gyro": False
}
