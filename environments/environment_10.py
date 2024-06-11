from classes.staticcircle import StaticCircle
from classes.staticpolygon import StaticPolygon
from classes.robot import Robot

obstacle_1 = StaticPolygon([(10, 10), (10, 610)], 5, 3, no_interference=2)
obstacle_2 = StaticPolygon([(130, 10), (130, 610)], 5, 3, no_interference=2)
obstacle_3 = StaticPolygon([(250, 10), (250, 610)], 5, 3, no_interference=2)
obstacle_4 = StaticPolygon([(10, 10), (250, 10)], 5, 3, no_interference=2)
obstacle_5 = StaticPolygon([(260, 10), (260, 610)], 5, 3, no_interference=2)
obstacle_6 = StaticPolygon([(400, 10), (400, 610)], 5, 3, no_interference=2)
obstacle_7 = StaticPolygon([(260, 300), (400, 300)], 5, 3, no_interference=2)
obstacle_8 = StaticPolygon([(260, 10), (400, 10)], 5, 3, no_interference=2)
obstacle_9 = StaticPolygon([(410, 10), (600, 10)], 5, 3, no_interference=2)
obstacle_10 = StaticPolygon([(410, 10), (410, 610)], 5, 3, no_interference=2)
obstacle_11 = StaticPolygon([(410, 300), (600, 300)], 5, 3, no_interference=2)
obstacle_12 = StaticPolygon([(600, 10), (600, 300)], 5, 3, no_interference=2)
obstacle_13 = StaticPolygon([(610, 10), (790, 10)], 5, 3, no_interference=2)
obstacle_14 = StaticPolygon([(610, 10), (610, 300)], 5, 3, no_interference=2)
obstacle_15 = StaticPolygon([(610, 300), (790, 300)], 5, 3, no_interference=2)
obstacle_16 = StaticPolygon([(790, 300), (790, 630)], 5, 3, no_interference=2)
obstacle_17 = StaticPolygon([(790, 630), (10, 630)], 5, 3, no_interference=2)


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
 ]

target = StaticCircle(700, 150, 5, 3)
agent = Robot(70, 45)

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 640
