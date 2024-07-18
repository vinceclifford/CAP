from src.staticpolygon import StaticPolygon
from src.staticcircle import StaticCircle
from src.robot import Robot

obstacle_1 = StaticPolygon([(400, 100), (400, 300)], 80, 3, )
obstacle_2 = StaticPolygon([(200, 100), (400, 100)], 80, 3, )
obstacle_3 = StaticPolygon([(200, 300), (400, 300)], 80, 3, )
obstacle_4 = StaticPolygon([(200, 131), (200, 300)], 80, 3)

obstacles = [
    obstacle_1, 
    obstacle_2, 
    obstacle_3,
    obstacle_4
]

target = StaticCircle(600, 200, 30, 3)
agent = Robot(300, 200)

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 640
