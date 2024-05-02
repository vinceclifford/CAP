from classes.static_circle import Static_Circle
from classes.static_polygon import Static_Polygon
from classes.robot import Robot

obstacle_1 = Static_Polygon([(10, 10), (10, 610), (10, 610)], 5, 3,no_interference=2) 
obstacle_2 = Static_Polygon([(130, 10), (130, 610), (130, 610)], 5, 3,no_interference=2)
obstacle_3 = Static_Polygon([(250, 10), (250, 610), (250, 610)], 5, 3, no_interference=2)
obstacle_4 = Static_Polygon([(10, 10), (250, 10), (250, 10)], 5, 3, no_interference=2)
obstacle_5 = Static_Polygon([(260, 10), (260, 610), (260, 610)], 5, 3, no_interference=2) 
obstacle_6 = Static_Polygon([(400, 10), (400, 610), (400, 610)], 5, 3, no_interference=2)
obstacle_7 = Static_Polygon([(260, 300), (400, 300), (400, 300)], 5, 3, no_interference=2)
obstacle_8 = Static_Polygon([(260, 10), (400, 10), (400, 10)], 5, 3, no_interference=2)
obstacle_9 = Static_Polygon([(410, 10), (600, 10), (600, 10)], 5, 3,no_interference=2) 
obstacle_10 = Static_Polygon([(410, 10), (410, 610), (410, 610)], 5, 3,no_interference=2)
obstacle_11 = Static_Polygon([(410, 300), (600, 300), (600, 300)], 5, 3,no_interference=2)
obstacle_12 = Static_Polygon([(600, 10), (600, 300), (600, 300)], 5, 3, no_interference=2)
obstacle_13 = Static_Polygon([(610, 10), (790, 10), (790, 10)], 5, 3, no_interference=2) 
obstacle_14 = Static_Polygon([(610, 10), (610, 300), (610, 300)], 5, 3,no_interference=2)
obstacle_15 = Static_Polygon([(610, 300), (790, 300), (790, 300)], 5, 3, no_interference=2)
obstacle_16 = Static_Polygon([(790, 300), (790, 630), (790, 630)], 5, 3, no_interference=2)
obstacle_17 = Static_Polygon([(790, 630), (10, 630), (10, 630)], 5, 3, no_interference=2)


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
    obstacle_17
}

target = Static_Circle(700, 150, 5, 3)
agent = Robot(70, 45)