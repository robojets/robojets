def Round1(robo):
    robo.lift.move(500,500)
    # ודחיפה של המכולה לעיגול נסיעה ישר לכיוון צפון
    robo.drive_straight(450,600,700)
    # פניה לכיוון מערב
    robo.turn(120)
    # חזרה לביית
    robo.drive_straight(350,600,900)
    