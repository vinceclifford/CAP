from src.staticpolygon import StaticPolygon
from src.staticcircle import StaticCircle
from src.robot import Robot

obstacle_1 = StaticCircle(150, 150, 100, 3, 50)
obstacle_2 = StaticPolygon([(310, 145), (360, 95), (410, 145)], 80, 4)
obstacle_3 = StaticPolygon([(330, 370), (400, 370), (400, 220), (330, 220)], 80, 6)
obstacle_4 = StaticPolygon([(160,260), (220, 265), (240, 295), (250, 340), (210, 370), (200, 310)], 60, 3)

obstacles = [
    obstacle_1,
    obstacle_2,
    obstacle_3,
    obstacle_4
]

agent = Robot(50, 450)
target = StaticCircle(450, 50, 30, 3)

SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
