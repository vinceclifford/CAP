import unittest
from classes.static_polygon import Static_Polygon

class Test_Static_Square(unittest.TestCase): 
    
    def test_square(self): 
        square = Static_Polygon(50, 3, [(0,0), (0,1), (2,1), (2,0)])
        self.assertEqual(square.get_shortest_distance_to_square(1,2), 1, "The distance should be one!")
    
if __name__ == '__main__': 
    unittest.main()