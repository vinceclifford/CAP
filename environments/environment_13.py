from classes.staticpolygon import StaticPolygon
from classes.staticcircle import StaticCircle
from classes.robot import Robot

obstacle_1 = StaticPolygon([(300, 300), (300, 400), (400, 400)], 5, 3, no_interference=0)

obstacles = [
    obstacle_1
]
target = StaticCircle(500, 500, 5, 3)
agent = Robot(75 , 45)