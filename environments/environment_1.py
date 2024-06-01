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
obstacle_20 = Static_Circle(350, 200, 90, 3)
obstacle_21 = Static_Circle(10, 400, 70, 2)
obstacle_22 = Static_Circle(450, 100, 110, 2.5)
obstacle_23 = Static_Circle(150, 250, 120, 3)
obstacle_24 = Static_Circle(250, 450, 80, 3)
obstacle_25 = Static_Circle(100, 100, 100, 3)
obstacle_26 = Static_Circle(400, 200, 130, 4)
obstacle_27 = Static_Circle(300, 400, 60, 3)
obstacle_28 = Static_Circle(200, 300, 150, 2)
obstacle_29 = Static_Circle(450, 350, 100, 3)
obstacle_30 = Static_Circle(10, 200, 80, 2.5)
obstacle_31 = Static_Circle(350, 450, 120, 3)
obstacle_32 = Static_Circle(250, 50, 90, 3)
obstacle_33 = Static_Circle(150, 150, 70, 3)
obstacle_34 = Static_Circle(450, 250, 110, 3)
obstacle_35 = Static_Circle(50, 350, 60, 3)
obstacle_36 = Static_Circle(350, 50, 100, 3)
obstacle_37 = Static_Circle(150, 450, 80, 3)
obstacle_38 = Static_Circle(450, 150, 120, 3)

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
}

target = Static_Circle(360, 360, 30, 3)
agent = Robot(45, 45)