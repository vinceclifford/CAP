from shapely.geometry import Polygon, Point

class Static_Polygon: 
    
    # We are going to store the edges of the square as follows: [(x_1, y_1), (x_2, y_2), (x_3, y_3)]
    def __init__(self, distance_of_influence, attraction, edges) -> None:
        self.distance_of_influence = distance_of_influence
        self.attraction = attraction 
        self.edges = edges
        self.polygon = Polygon(edges)
         
        
    def get_shortest_distance_to_square(self, x_robot, y_robot): 
        point = Point(x_robot, y_robot)
        distance = point.distance(self.polygon)
        return distance
            
        