# Environment to test the implementation of Squares via Static_Polygon

from classes.staticcircle import StaticCircle
from classes.staticpolygon import StaticPolygon
from classes.robot import Robot

obstacle_1 = StaticCircle(200, 200, 80, 3)
obstacle_2 = StaticPolygon([(300, 300), (350, 300), (350, 400), (300, 400)], 80, 3)

target = StaticCircle(400, 400, 30, 3)
agent = Robot(45, 45)

obstacles = [
    obstacle_1, 
    obstacle_2
]