from classes.staticcircle import StaticCircle
from classes.robot import Robot

# Test case if we are trapped. We should not find a path to the patient. 

obstacle_1 = StaticCircle(200, 100, 130, 3, no_interference=50)
obstacle_2 = StaticCircle(100, 200, 130, 3, no_interference=50)
obstacle_3 = StaticCircle(300, 200, 130, 3, no_interference=50)
obstacle_4 = StaticCircle(200, 300, 130, 3, no_interference=50)
obstacle_5 = StaticCircle(266, 266, 130, 3, no_interference=50)
obstacle_6 = StaticCircle(144, 266, 130, 3, no_interference=50)
obstacle_7 = StaticCircle(266, 144, 130, 3, no_interference=50)
obstacle_8 = StaticCircle(144, 144, 130, 3, no_interference=50)


obstacles = [
    obstacle_1, 
    obstacle_2,
    obstacle_3, 
    obstacle_4, 
    obstacle_5, 
    obstacle_6, 
    obstacle_7, 
    obstacle_8
]

target = StaticCircle(500, 500, 30, 3)
agent = Robot(200, 200)

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 640
