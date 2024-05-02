from classes.static_polygon import Static_Polygon
from classes.static_circle import Static_Circle
from classes.robot import Robot 

obstacle_1 = Static_Polygon([(0, 0), (800, 0), (800, 0)], 80, 3 )
obstacle_2 = Static_Polygon([(0,640), (800, 640), (800,640)], 80, 3 )
obstacle_3 = Static_Polygon([(0,0), (0, 640), (0,640)], 80, 3 )
obstacle_4 = Static_Polygon([(800, 0), (800, 640), (800, 640)], 80, 3)
obstacle_5 = Static_Polygon([(267, 0), (267, 256),(267, 256)], 80, 3 )
obstacle_6 = Static_Polygon([(400,0), (400, 384), (400,384)], 80, 3 )
obstacle_7 = Static_Polygon([(533,128), (667, 128), (667, 128)], 80, 3 )
obstacle_8 = Static_Polygon([(133, 128), (133, 384), (133, 384)], 80, 3)
obstacle_9 = Static_Polygon([(133, 384), (400, 384), (400, 384)], 80, 3 )
obstacle_10 = Static_Polygon([(533,256), (533, 384), (533, 384)], 80, 3 )
obstacle_11 = Static_Polygon([(667,128), (667, 490), (667, 490)], 80, 3 )
obstacle_12 = Static_Polygon([(133,512), (667, 512), (667, 512)], 80, 3 )
obstacle_13 = Static_Polygon([(533,512), (533, 640), (533, 640)], 80, 3 )

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
    obstacle_13
}

target = Static_Circle(667, 610, 30, 3)
agent = Robot(50, 50)
