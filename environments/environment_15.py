from src.staticpolygon import StaticPolygon
from src.staticcircle import StaticCircle
from src.robot import Robot

obstacle_1 = StaticPolygon([(0, 50), (100, 130), (0, 130)], 80, 4)
obstacle_2 = StaticPolygon([(135, 80), (205, 50), (180, 80), (205, 110)], 80, 4)
obstacle_3 = StaticPolygon([(310, 52), (445, 52), (445, 80), (410, 80), (377, 120),(345, 80), (310, 80)], 80, 4)
obstacle_4 = StaticCircle(435, 215, 100, 4, no_interference=50)
obstacle_5 = StaticPolygon([(280, 275), (370, 275), (410, 320), (370, 365), (280, 365), (240, 320)], 80, 3)
obstacle_6 = StaticPolygon([(310, 400), (500, 400), (500, 460), (310, 460)], 100, 4)
obstacle_7 = StaticPolygon([(90, 398), (270, 398), (270, 460), (90, 460)], 100, 4)
obstacle_8 = StaticPolygon([(0, 280), (180, 280), (180, 360), (0, 360)], 100, 4)
obstacle_9 = StaticPolygon([(40, 170), (275, 170), (275, 225), (40, 225)], 80, 4)
obstacle_10 = StaticPolygon([(255, 90), (255, 120), (290, 150), (315, 150), (310, 110)], 30, 4)

obstacles = [obstacle_1, obstacle_2, obstacle_3, obstacle_4, obstacle_5, obstacle_6, obstacle_7, obstacle_8, obstacle_9, obstacle_10]

agent = Robot(50, 450)
target = StaticCircle(470, 30, 30, 3)

SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
