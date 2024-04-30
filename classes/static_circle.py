import numpy as np  

class Static_Circle: 
    def __init__(self, x, y, radius,  attraction, no_interference=10) -> None:
        self.vektor = (x,y)
        self.distance_of_influence = radius
        self.attraction = attraction 
        self.no_interference = no_interference
        
    def distance(self, x_robot, y_robot): 
        return np.linalg.norm(np.array([self.vektor[0], self.vektor[1]]) - np.array([x_robot, y_robot]))