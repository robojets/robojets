import time
def Round2(robo):
    robo.lift.move(-500,500)
    robo.drive_stright(300,7000,400)
    robo.turn(-90)
    robo.drive_stright(260+850,7000,500)
    robo.drive_speed(100,250,-150)
    robo.drive_stright(100,250,250)
    robo.turn(-36)
    robo.drive_stright(-80,250,250)
    robo.lift.move(500,480)
    robo.drive_stright(-120,250,250)
    robo.lift.move(-500,480)

    