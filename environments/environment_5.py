from classes.static_circle import Static_Circle
from classes.static_polygon import Static_Polygon
from classes.robot import Robot

obstacle_1 = Static_Polygon([(50, 50), (250, 50), (250, 150), (50, 150)], 100, 3)
obstacle_2 = Static_Polygon([(200, 100), (350, 100), (350, 250), (200, 250)], 80, 3)
obstacle_3 = Static_Polygon([(100, 200), (250, 200), (250, 350), (100, 350)], 90, 3)
obstacle_4 = Static_Polygon([(300, 300), (450, 300), (450, 450), (300, 450)], 110, 3)
obstacle_5 = Static_Polygon([(150, 400), (300, 400), (300, 550), (150, 550)], 80, 3)
obstacle_6 = Static_Polygon([(400, 500), (550, 500), (550, 650), (400, 650)], 120, 3)

obstacles = {
    obstacle_1,
    obstacle_2, 
    obstacle_3,
    obstacle_4,
    obstacle_5, 
    obstacle_6
}

target = Static_Circle(261, 375, 30, 3)
agent = Robot(20, 20)
