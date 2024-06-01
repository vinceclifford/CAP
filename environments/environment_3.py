from classes.staticcircle import StaticCircle
from classes.staticpolygon import StaticPolygon
from classes.robot import Robot

obstacle_1 = StaticCircle(200, 200, 80, 3)
obstacle_2 = StaticCircle(120, 140, 80, 3)
obstacle_3 = StaticCircle(170, 120, 90, 2)
obstacle_4 = StaticCircle(300, 300, 100, 3)
obstacle_5 = StaticPolygon([(299, 299), (350, 300), (350, 400), (300, 400)], 80, 3)

target = StaticCircle(400, 400, 30, 3)
agent = Robot(45, 45)

obstacles = [
    obstacle_1, 
    obstacle_2, 
    obstacle_3, 
    obstacle_4, 
    obstacle_5
]