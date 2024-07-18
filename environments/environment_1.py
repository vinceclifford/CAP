from src.staticcircle import StaticCircle
from src.robot import Robot

obstacle_1 = StaticCircle(200, 200, 130, 3)
obstacle_2 = StaticCircle(120, 140, 80, 3)
obstacle_3 = StaticCircle(170, 120, 100, 2)
obstacle_4 = StaticCircle(300, 300, 120, 3)
obstacle_5 = StaticCircle(270, 300, 90, 2.5)
obstacle_6 = StaticCircle(400, 375, 140, 3)
obstacle_7 = StaticCircle(100, 277, 90, 3)
obstacle_8 = StaticCircle(420, 430, 50, 3)
obstacle_9 = StaticCircle(400, 60, 130, 4)
obstacle_11 = StaticCircle(200, 1, 10, 3)
obstacle_12 = StaticCircle(200, 2, 10, 3)
obstacle_13 = StaticCircle(200, 4, 10, 3)
obstacle_14 = StaticCircle(200, 8, 10, 3)
obstacle_15 = StaticCircle(200, 16, 10, 3)
obstacle_16 = StaticCircle(200, 32, 10, 3)
obstacle_17 = StaticCircle(200, 64, 10, 3)
obstacle_18 = StaticCircle(200, 128, 10, 3)
obstacle_19 = StaticCircle(200, 256, 10, 3)
obstacle_10 = StaticCircle(200, 512, 10, 3)
obstacle_20 = StaticCircle(350, 200, 90, 3)
obstacle_21 = StaticCircle(10, 400, 70, 2)
obstacle_22 = StaticCircle(450, 100, 110, 2.5)
obstacle_23 = StaticCircle(150, 250, 120, 3)
obstacle_24 = StaticCircle(250, 450, 80, 3)
obstacle_25 = StaticCircle(100, 100, 100, 3)
obstacle_26 = StaticCircle(400, 200, 130, 4)
obstacle_27 = StaticCircle(300, 400, 60, 3)
obstacle_28 = StaticCircle(200, 300, 150, 2)
obstacle_29 = StaticCircle(450, 350, 100, 3)
obstacle_30 = StaticCircle(10, 200, 80, 2.5)
obstacle_31 = StaticCircle(350, 450, 120, 3)
obstacle_32 = StaticCircle(250, 50, 90, 3)
obstacle_33 = StaticCircle(150, 150, 70, 3)
obstacle_34 = StaticCircle(450, 250, 110, 3)
obstacle_35 = StaticCircle(50, 350, 60, 3)
obstacle_36 = StaticCircle(350, 50, 100, 3)
obstacle_37 = StaticCircle(150, 450, 80, 3)
obstacle_38 = StaticCircle(450, 150, 120, 3)

obstacles = [
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
    obstacle_19,
    obstacle_20,
    obstacle_21,
    obstacle_22,
    obstacle_23,
    obstacle_24,
    obstacle_25,
    obstacle_26,
    obstacle_27,
    obstacle_28,
    obstacle_29,
    obstacle_30,
    obstacle_31,
    obstacle_32,
    obstacle_33,
    obstacle_34,
    obstacle_35,
    obstacle_36,
    obstacle_37,
    obstacle_38
]

target = StaticCircle(360, 360, 30, 3)
agent = Robot(45, 45)

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 640
