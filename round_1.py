import time

def Round1(robo):
   robo.stop()
   robo.settings(turn_rate=150, turn_acceleration=150)
   # מעלית יורדת למטה בתחילת הסבב
   robo.lift.move(-460,500)
   
   # הרובט נוסע לכיוון המנוע
   robo.drive_straight(645,400,400)
   robo.turn(60)
   robo.drive_straight(215,400,400)

   # הרובוט הופך את המנוע
   robo.lift.move(250,600)
   robo.turn(10) 
   robo.lift.move(200,600)

   # הרובוט חוזר לביית תוך כדי הזזה של המכולה הכחולה לתוך הביית
   robo.turn(-18)
   # robo.turn(25)
   robo.drive_straight(-780,400,400)
   robo.turn(-50)

   robo.stop()
   return            


  