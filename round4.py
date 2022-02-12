import time

def Round4(robo):
   robo.straight(400)
   robo.lift.move(-700,7000)
   robo.straight(-70)
   robo.lift.move(700,10000)
   time.sleep(2)
   robo.turn(-50)
   robo.straight(230)
   robo.turn(-40)
   robo.straight(-750)
  