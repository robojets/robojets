def Round3(robo):
    # נסיעה ישרה לכיוון מזרח לעיגול של התרנגול
    robo.drive_straight(510,500,400)
    # נסיעה ישרה לאחור בחזרה לבית
    robo.drive_straight(-900,500,700)
    robo.turn(-95)

    robo.stop()
    return