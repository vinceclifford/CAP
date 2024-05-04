from shapely.geometry import Polygon, Point

class Static_Polygon: 
    
   
    def __init__(self, vertices, distance_of_influence, attraction, no_interference=10) -> None:
        """
        Args:
            vertices (list): list of tuples with two entries each. A x and a y coordinate
            distance_of_influence (int): Until which distance does 
            attraction (int): How much influence does the polygon got. A "typical" value for this attribute is 100, the bigger the value 
            the more infcluence it will have
            no_interference (int, optional): Distance around polygon that the robot must not interfere with the polygon. Defaults to 10.
        """
        self.distance_of_influence = distance_of_influence
        self.attraction = attraction 
        self.vertices = vertices
        #self.polygon = Polygon(vertices)
        self.no_interference = no_interference
         
        """
    def distance(self, x_robot, y_robot): 
        point = Point(x_robot, y_robot)
        distance = point.distance(self.polygon)
        return distance
        """
            
        