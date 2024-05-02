from classes.static_polygon import Static_Polygon
from classes.static_circle import Static_Circle
from classes.robot import Robot 

obstacle_1 = Static_Polygon([(400, 100), (400, 300), (400, 300)], 80, 3, )
obstacle_2 = Static_Polygon([(200,100), (400, 100), (400,100)], 80, 3, )
obstacle_3 = Static_Polygon([(200,300), (400, 300), (400,300)], 80, 3, )
obstacle_4 = Static_Polygon([(200, 131), (200, 300), (200, 300)], 80, 3)

obstacles = {
    obstacle_1, 
    obstacle_2, 
    obstacle_3,
    obstacle_4
}

target = Static_Circle(600, 200, 30, 3)
agent = Robot(300, 200)
