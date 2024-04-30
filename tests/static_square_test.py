import unittest
from classes.static_polygon import Static_Polygon

class Test_Static_Square(unittest.TestCase): 
    
    def test_square_01(self): 
        square = Static_Polygon( [(0,0), (0,1), (2,1), (2,0)], 50, 3)
        self.assertEqual(square.distance(1,2), 1, "The distance should be 1!")
        
    def test_square_02(self): 
        square = Static_Polygon([(0,0), (0,1), (2,1), (2,0)], 50, 3)
        self.assertEqual(square.distance(1,0.5), 0, "The distance should be 0!")
    
if __name__ == '__main__': 
    unittest.main() 