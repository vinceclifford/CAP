from src.staticpolygon import StaticPolygon
from src.staticcircle import StaticCircle
from src.robot import Robot

obstacle_1 = StaticPolygon([(360, 375), (360, 410), (425, 410), (425, 375)], 60, 3)
obstacle_2 = StaticPolygon([(380, 275), (380, 340), (420, 340), (420, 275)], 60, 3)
obstacle_3 = StaticPolygon([(295, 275), (295, 240), (335, 240), (335, 275)], 60, 3)
obstacle_4 = StaticPolygon([(185, 410), (250, 410), (250, 375), (185, 375)], 60, 3)
obstacle_5 = StaticPolygon([(185, 290), (185, 325), (250, 325), (250, 290)], 60, 3)
obstacle_6 = StaticPolygon([(185, 130), (250, 130), (250, 95), (185, 95)], 60, 3)
obstacle_7 = StaticPolygon([(75, 235), (140, 235), (140, 200), (75, 200)], 60, 3)
obstacle_8 = StaticPolygon([(280, 235), (345, 235), (345, 200), (280, 200)], 60, 3)
obstacle_9 = StaticPolygon([(295, 75), (295, 40), (335, 40), (335, 75)], 60, 3)
obstacle_10 = StaticPolygon([(85, 75), (85, 145), (125, 145), (125, 75)], 60, 3)
obstacle_11 = StaticPolygon([(380, 180), (380, 240), (420, 240), (420, 180)], 60, 3)
obstacle_12 = StaticPolygon([(75, 275), (75, 340), (140, 340), (140, 275)], 60, 3)
obstacle_13 = StaticPolygon([(200, 180), (200, 240), (240, 240), (240, 180)], 60, 3)

obstacles = [
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
]

agent = Robot(50, 450)
target = StaticCircle(440, 30, 30, 3)

SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
