def Round2(robo):
    # המעלית עולה בתחילת הסבב
    robo.lift.up(500)

    # הרובוט נוסע לכיוון המטס
    robo.drive_straight(480,600,700)

    # המעלית מורידה את המכולה
    robo.lift.move(-550, 600)

    # הרובוט חוזר הבייתה
    robo.drive_straight(-60,600,700)
    robo.turn(-120)
    robo.drive_straight(-350,600,900)
    robo.turn(-50)
    