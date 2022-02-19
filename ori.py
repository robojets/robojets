import time
def Round2(robo):
    robo.lift.move(-500,500)
    robo.drive_straight(300,7000,400)
    robo.turn(-99)
    robo.drive_straight(260+900,7000,500)
    robo.turn(-70)
    robo.drive_straight(200,250,250)
    robo.turn(-110)
    robo.drive_straight(-130,250,250)
    robo.drive_straight(100,250,250)
    robo.lift.move(500,480)
    robo.drive_straight(-150,250,500)
    robo.turn(-13)
    robo.drive_straight(-350,7000,250)
    return              

    