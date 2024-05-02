#Test für Wanddicke testen

from classes.static_circle import Static_Circle
from classes.static_polygon import Static_Polygon
from classes.robot import Robot

obstacle_1 = Static_Circle(200, 1, 10, 3)
obstacle_2 = Static_Circle(200, 2, 10, 3)
obstacle_3 = Static_Circle(200, 4, 10, 3)
obstacle_4 = Static_Circle(200, 8, 10, 3)
obstacle_5 = Static_Circle(200, 16, 10, 3)
obstacle_6 = Static_Circle(200, 32, 10, 3)
obstacle_7 = Static_Circle(200, 64, 10, 3)
obstacle_8 = Static_Circle(200, 128, 10, 3)
obstacle_9 = Static_Circle(200, 256, 10, 3)
obstacle_10 = Static_Circle(200, 512, 10, 3)

obstacle_11 = Static_Polygon([(400, 1), (500, 1), (500, 1)], 10, 3)
obstacle_12 = Static_Polygon([(400, 2), (500, 2), (500, 2)], 10, 3)
obstacle_13 = Static_Polygon([(400, 4), (500, 4), (500, 4)], 10, 3)
obstacle_14 = Static_Polygon([(400, 8), (500, 8), (500, 8)], 10, 3)
obstacle_15 = Static_Polygon([(400, 16), (500, 16), (500, 16)], 10, 3)
obstacle_16 = Static_Polygon([(400, 32), (500, 32), (500, 32)], 10, 3)
obstacle_17 = Static_Polygon([(400, 64), (500, 64), (500, 64)], 10, 3)
obstacle_18 = Static_Polygon([(400, 128), (500, 128), (500, 128)], 10, 3)
obstacle_19 = Static_Polygon([(400, 256), (500, 256), (500, 256)], 10, 3)
obstacle_20 = Static_Polygon([(400, 512), (500, 512), (500, 512)], 10, 3)

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
    obstacle_20
}

target = Static_Circle(780, 620, 30, 3)
agent = Robot(20, 20)