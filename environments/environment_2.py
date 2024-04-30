# Environment to test the implementation of Squares via Static_Polygon

from classes.static_circle import Static_Circle
from classes.static_polygon import Static_Polygon
from classes.robot import Robot

obstacle_1 = Static_Circle(200, 200, 80, 3)
obstacle_2 = Static_Polygon([(300, 300), (350, 300), (350, 400), (300, 400)], 80, 3)

target = Static_Circle(400, 400, 30, 3)
agent = Robot(45, 45)

obstacles = {
    obstacle_1, 
    obstacle_2
}