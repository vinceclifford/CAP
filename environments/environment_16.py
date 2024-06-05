from classes.staticpolygon import StaticPolygon
from classes.staticcircle import StaticCircle
from classes.robot import Robot

obstacle_1 = StaticPolygon([(0, 425), (30,425), (30, 145), (85, 145), (85, 85), (30, 85), (30, 50), (360, 50), (360, 20), (0, 20)], 80, 4)
obstacle_2 = StaticPolygon([(100, 475), (425, 475), (425, 360), (445, 360), (445, 285), (425, 285),(425, 85), (395, 85), (395, 120), (345, 120), (345, 205), (395, 205), (395, 280), (355, 280), (355, 360), (395, 360), (395, 440), (100, 440)], 80, 4)
obstacle_3 = StaticPolygon([(125, 80), (190, 80), (190, 150), (125, 150)], 80, 4)
obstacle_4 = StaticPolygon([(260, 420), (345, 420), (345, 395), (260, 395)], 50, 3)
obstacle_5 = StaticPolygon([(290, 310), (290, 270), (225, 270), (225, 310)], 50, 3)
obstacle_6 = StaticPolygon([(120, 400), (120, 310), (150, 310), (150, 275), (85, 275), (85, 400)], 60, 3)
obstacle_7 = StaticPolygon([(180, 370), (290, 370), (290, 335), (180, 335)], 100, 3)
obstacle_8 = StaticPolygon([(150, 245), (150, 185), (90, 185), (90, 245)], 60, 3)
obstacle_9 = StaticPolygon([(285, 230), (285, 180), (260, 180), (260, 120), (295, 120), (295, 70), (220, 70), (220, 180),(195, 180), (195, 230)], 60, 3)
obstacle_10 = StaticPolygon([(365, 260), (365, 230), (330, 230), (330, 260)], 60, 3)

obstacles = [
    obstacle_1,
    obstacle_2,
    obstacle_3,
    obstacle_4,
    obstacle_5,
    obstacle_6,
    obstacle_7,
    obstacle_8,
    obstacle_9,
    obstacle_10,
]

agent = Robot(50, 450)
target = StaticCircle(440, 30, 30, 3)