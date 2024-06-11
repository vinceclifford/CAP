#Test für Wanddicke testen

from classes.staticcircle import StaticCircle
from classes.staticpolygon import StaticPolygon
from classes.robot import Robot

obstacle_1 = StaticCircle(200, 1, 10, 3)
obstacle_2 = StaticCircle(200, 2, 10, 3)
obstacle_3 = StaticCircle(200, 4, 10, 3)
obstacle_4 = StaticCircle(200, 8, 10, 3)
obstacle_5 = StaticCircle(200, 16, 10, 3)
obstacle_6 = StaticCircle(200, 32, 10, 3)
obstacle_7 = StaticCircle(200, 64, 10, 3)
obstacle_8 = StaticCircle(200, 128, 10, 3)
obstacle_9 = StaticCircle(200, 256, 10, 3)
obstacle_10 = StaticCircle(200, 512, 10, 3)

obstacle_11 = StaticPolygon([(400, 1), (500, 1)], 10, 3)
obstacle_12 = StaticPolygon([(400, 2), (500, 2)], 10, 3)
obstacle_13 = StaticPolygon([(400, 4), (500, 4)], 10, 3)
obstacle_14 = StaticPolygon([(400, 8), (500, 8)], 10, 3)
obstacle_15 = StaticPolygon([(400, 16), (500, 16)], 10, 3)
obstacle_16 = StaticPolygon([(400, 32), (500, 32)], 10, 3)
obstacle_17 = StaticPolygon([(400, 64), (500, 64)], 10, 3)
obstacle_18 = StaticPolygon([(400, 128), (500, 128)], 10, 3)
obstacle_19 = StaticPolygon([(400, 256), (500, 256)], 10, 3)
obstacle_20 = StaticPolygon([(400, 512), (500, 512)], 10, 3)

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
    obstacle_11, 
    obstacle_12,
    obstacle_13,
    obstacle_14, 
    obstacle_15,
    obstacle_16,
    obstacle_17, 
    obstacle_18,
    obstacle_19,
    obstacle_20
]

target = StaticCircle(780, 620, 30, 3)
agent = Robot(20, 20)

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 640
