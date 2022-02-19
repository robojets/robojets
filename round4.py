import time

def Round4(robo):
   #robo.run_until_line(1)
   #robo.follow_line(1, 0)
   robo.lift.move(-460,500)
   robo.drive_straight(645,400,400)
   robo.turn(70)
   robo.drive_straight(215,400,400)
   robo.lift.move(250,600)
   robo.turn(10) 
   robo.lift.move(200,600)
   robo.turn(15)
   robo.turn(-40)
   robo.drive_straight(-780,400,400)
   robo.turn(-50)

  