from classes.staticcircle import StaticCircle
from classes.staticpolygon import StaticPolygon
from classes.robot import Robot 


obstacle_1 = StaticPolygon([(100, 200), (125, 200), (125, 600), (600, 600), (600, 625), (100, 625)], 80, 3)
obstacle_2 = StaticPolygon([(175, 250), (600, 250), (600, 550), (175, 550)], 80, 3)
obstacle_3 = StaticCircle(100, 100, 50, 3)
obstacle_4 = StaticCircle(50, 140, 50, 3)
obstacle_5 = StaticCircle(160, 220, 200, 4, no_interference=20)


obstacles = [
    obstacle_1, 
    obstacle_2, 
    obstacle_3,
    obstacle_4,
    obstacle_5
]

target = StaticCircle(300, 575, 30, 3)
agent = Robot(45, 45)

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 640
