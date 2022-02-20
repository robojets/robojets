def Round2(robo):
    robo.stop()
    robo.settings(turn_rate=150, turn_acceleration=150)

    # ודחיפה של המכולה לעיגול נסיעה ישר לכיוון צפון
    robo.drive_straight(450,600,700)
    # פניה לכיוון מערב
    robo.turn(70)
    # חזרה לביית
    robo.drive_straight(-350,600,900)
    robo.turn(-70)

    robo.stop()
    return            
   