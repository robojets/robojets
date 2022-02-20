import time
def Round4(robo):
    robo.stop()
    robo.settings(turn_rate=150, turn_acceleration=150)

    # מוריד את המעלית עד למטה בתחילת הסבב
    robo.lift.move(-500, 500)

    # נוסע לכיוון הגשר
    robo.drive_straight(340,500,400)
    robo.turn(86)
    robo.drive_straight(260+935,500,500) # נוסע עד קרגו קונקט ומפיל גשר ומסיע משאית 

    # פונה לעיגול של הקרגו קוקט ומציב את המכולות במקום המתאים
    robo.turn(79)
    robo.drive_straight(200,250,250)
    robo.turn(85)
    robo.drive_straight(-140,250,250)
    robo.drive_straight(80,250,250)
    robo.lift.move(500,480) # מרים את המעלית כדי שהמכולותישארו במקום

    # הרובוט נובע לכיוון המסוק לשחחר את המטען
    robo.drive_straight(-150,250,500)
    robo.turn(-25)
    robo.drive_straight(-350,400,250) # מסתובב שמאלה ופוגע במסוק 
    robo.stop()
    return            

    