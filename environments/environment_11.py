from classes.staticcircle import StaticCircle
from classes.robot import Robot

obstacle_1 = StaticCircle(20, 20, 80, 3, no_interference=0)
target = StaticCircle(400, 400, 80, 3)

obstacles = [obstacle_1]
agent = Robot(750, 620)
