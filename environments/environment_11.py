from classes.static_circle import Static_Circle    
from classes.robot import Robot

obstacle_1 = Static_Circle(20, 20, 80, 3, no_interference=0)
target = Static_Circle(400,400,80,3)

obstacles = {obstacle_1}
agent = Robot(750, 620)
