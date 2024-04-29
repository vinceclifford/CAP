# Environment to test the implementation of Squares via Static_Polygon

from classes.static_circle import Static_Circle
from classes.static_polygon import Static_Polygon

obstacle_1 = Static_Circle(200, 200, 80, 3)
obstacle_2 = Static_Polygon([(300, 300), (350, 300), (350, 400), (300, 400)], 80, 3)

obstacles = {
    obstacle_1, 
    obstacle_2
}