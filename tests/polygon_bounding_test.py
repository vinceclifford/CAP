import unittest
from classes.staticpolygon import StaticPolygon


class PolygonBoundingTest(unittest.TestCase):
    def test_polygon(self):
        polygon = StaticPolygon([(0, 2), (2, 4), (4, 2), (2, 0)], 130, 3)

        if polygon.x_max != 4 or polygon.y_max != 4 or polygon.x_min != 0 or polygon.y_min != 0:
            self.fail('Polygon not properly bounded')
