from classes.static_circle import Static_Circle
from classes.robot import Robot

# Test case if we are trapped. We should not find a path to the patient. 

obstacle_1 = Static_Circle(200, 100, 130, 3, no_interference=50)
obstacle_2 = Static_Circle(100, 200, 130, 3, no_interference=50)
obstacle_3 = Static_Circle(300, 200, 130, 3, no_interference=50)
obstacle_4 = Static_Circle(200, 300, 130, 3, no_interference=50)
obstacle_5 = Static_Circle(266, 266, 130, 3, no_interference=50)
obstacle_6 = Static_Circle(144, 266, 130, 3, no_interference=50)
obstacle_7 = Static_Circle(266, 144, 130, 3, no_interference=50)
obstacle_8 = Static_Circle(144, 144, 130, 3, no_interference=50)





obstacles = {
    obstacle_1, 
    obstacle_2,
    obstacle_3, 
    obstacle_4, 
    obstacle_5, 
    obstacle_6, 
    obstacle_7, 
    obstacle_8
}

target = Static_Circle(500, 500, 30, 3)
agent = Robot(200, 200)