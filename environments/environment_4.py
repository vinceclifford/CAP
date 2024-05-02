from classes.static_circle import Static_Circle
from classes.static_polygon import Static_Polygon
from classes.robot import Robot 


obstacle_1 = Static_Polygon([(100, 200), (125, 200), (125, 600),  (600, 600), (600,625), (100, 625)], 80, 3)
obstacle_2 = Static_Polygon([(175, 250), (600, 250), (600, 550), (175,550)], 80, 3)
obstacle_3 = Static_Circle(100, 100, 50, 3)
obstacle_4 = Static_Circle(50, 140, 50, 3)
obstacle_5 = Static_Circle(160, 220, 200, 4) 

obstacles = {
    obstacle_1, 
    obstacle_2, 
    obstacle_3,
    obstacle_4,
    obstacle_5
}

target = Static_Circle(300, 575, 30, 3)
agent = Robot(45, 45)