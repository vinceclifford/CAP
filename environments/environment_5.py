from classes.staticcircle import StaticCircle
from classes.staticpolygon import StaticPolygon
from classes.robot import Robot

obstacle_1 = StaticPolygon([(50, 50), (250, 50), (250, 150), (50, 150)], 100, 3)
obstacle_2 = StaticPolygon([(200, 100), (350, 100), (350, 250), (200, 250)], 80, 3)
obstacle_3 = StaticPolygon([(100, 200), (250, 200), (250, 350), (100, 350)], 90, 3)
obstacle_4 = StaticPolygon([(300, 300), (450, 300), (450, 450), (300, 450)], 110, 3)
obstacle_5 = StaticPolygon([(150, 400), (300, 400), (300, 550), (150, 550)], 80, 3)
obstacle_6 = StaticPolygon([(400, 500), (550, 500), (550, 650), (400, 650)], 120, 3)

obstacles = [
    obstacle_1,
    obstacle_2, 
    obstacle_3,
    obstacle_4,
    obstacle_5, 
    obstacle_6
]

target = StaticCircle(261, 375, 30, 3)
agent = Robot(20, 20)

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 640
