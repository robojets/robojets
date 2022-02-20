import time
def Round5(robo):

    robo.stop()
    robo.settings(turn_rate=150, turn_acceleration=150)

    robo.drive_straight(50,500,400)
    robo.turn(65)
    robo.drive_straight(450,500,400)
    robo.turn(-15)
    robo.drive_straight(500,500,400)
    robo.turn(45)
    robo.drive_straight(460,500,400)
    robo.drive_straight(-440,500,400)
    robo.turn(95)
    #robo.straight()
    robo.drive_straight(-185,500,400)
    # robo.drive_straight(10,500,400)
    robo.drive_to_line()
    robo.drive_straight(-30,500,400)
    robo.turn(120)
    robo.drive_to_line(sensor=1, color=1)
    #robo.drive_straight(10,200,200)
    #robo.drive_straight(-10,200,200)
    #robo.drive_straight(130,200,200)

    robo.stop()
    return

    robo.drive_straight(100,500,400)
    robo.turn(-90)
    return

    
    robo.drive_straight(275,500,400)
    return
    robo.turn(14)
    robo.drive_straight(300,500,400)
    #robo.drive_
    #robo.drive_to_line()
    #robo.follow_line(distance=500)
    robo.stop()
    return


    # מוריד מעלה את המעלית עד למעלה בתחילת הסבב
    robo.lift.up(500)

    # נוסע לכיוון הגשר
    robo.drive_straight(110,500,400)
    robo.turn(35)
    robo.drive_straight(260,500,500) # נוסע עד קרגו קונקט ומפיל גשר ומסיע משאית 
    robo.drive_straight(250,7000,500) # נוסע עד קרגו קונקט ומפיל גשר ומסיע משאית 
    robo.drive_straight(500,7000,500) # נוסע עד קרגו קונקט ומפיל גשר ומסיע משאית 

    # דוחף את המנוף
    robo.lift.move(-500,480) # מרים את המעלית כדי שהמכולותישארו במקום
    robo.drive_straight(70,7000,500) # נוסע עד קרגו קונקט ומפיל גשר ומסיע משאית 

    # נוסע לאחור לעמדת החניה
    robo.drive_straight(-300,250,250)
    robo.turn(80)
    robo.drive_straight(-85,250,250)
    robo.turn(-80)

    # דוחף את עמדת מניעת ההתנגשות ועוצר
    robo.drive_straight(-45,250,250)   
    