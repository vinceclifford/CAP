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
obstacle_11 = Static_Circle(200, 1, 10, 3)
obstacle_12 = Static_Circle(200, 2, 10, 3)
obstacle_13 = Static_Circle(200, 4, 10, 3)
obstacle_14 = Static_Circle(200, 8, 10, 3)
obstacle_15 = Static_Circle(200, 16, 10, 3)
obstacle_16 = Static_Circle(200, 32, 10, 3)
obstacle_17 = Static_Circle(200, 64, 10, 3)
obstacle_18 = Static_Circle(200, 128, 10, 3)
obstacle_19 = Static_Circle(200, 256, 10, 3)
obstacle_10 = Static_Circle(200, 512, 10, 3)

obstacles = {
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
    obstacle_19
}

target = Static_Circle(360, 360, 30, 3)
agent = Robot(45, 45)