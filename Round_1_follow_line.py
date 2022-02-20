def Round1_follow_line(robo):
    # מעלית יורדת למטה
    robo.lift.down(500)

    # סע לכיוון המנוע לאורך הקו
    robo.drive_to_line(color=0)
    robo.follow_line(distance=860)
    robo.drive_straight(40,200,200)

    # הפוך את המנוע
    robo.lift.move(250,600)
    robo.turn(10) 
    robo.lift.move(200,600)

    # חזוא לביית ודחוף את המכולה הכחולה
    robo.turn(-15)
    robo.drive_straight(-780,400,400)
    robo.turn(-50)
