from shapely.geometry import Polygon, Point

class Static_Polygon: 
    
    # We are going to store the vertices of the square as follows: [(x_1, y_1), (x_2, y_2), (x_3, y_3)]
    def __init__(self, vertices, distance_of_influence, attraction, no_interference=10) -> None:
        self.distance_of_influence = distance_of_influence
        self.attraction = attraction 
        self.vertices = vertices
        self.polygon = Polygon(vertices)
        self.no_interference = no_interference
         
        
    def distance(self, x_robot, y_robot): 
        point = Point(x_robot, y_robot)
        distance = point.distance(self.polygon)
        return distance
            
        