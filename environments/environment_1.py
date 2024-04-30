from classes.static_circle import Static_Circle
from classes.robot import Robot

obstacle_1 = Static_Circle(200, 200, 130, 3)
obstacle_2 = Static_Circle(120, 140, 80, 3)
obstacle_3 = Static_Circle(170, 120, 100, 2)
obstacle_4 = Static_Circle(300, 300, 120, 3)
obstacle_5 = Static_Circle(270, 300, 90, 2.5)
obstacle_6 = Static_Circle(400, 375, 140, 3)
obstacle_7 = Static_Circle(100, 277, 90, 3)
obstacle_8 = Static_Circle(420, 430, 50, 3)
obstacle_9 = Static_Circle(400, 60, 130, 4)

obstacles = {
    obstacle_1, 
    obstacle_2,
    obstacle_3, 
    obstacle_4, 
    obstacle_5, 
    obstacle_6, 
    obstacle_7, 
    obstacle_8,
    obstacle_9
}

target = Static_Circle(400, 400, 30, 3)
agent = Robot(45, 45)