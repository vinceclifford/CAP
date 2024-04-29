import numpy as np  

class Static_Circle: 
    def __init__(self, x, y, radius,  attraction) -> None:
        self.vektor = (x,y)
        self.radius_of_influence = radius
        self.attraction = attraction 
        
    def distance(self, x_robot, y_robot): 
        return np.linalg.norm(np.array([self.vektor[0], self.vektor[1]]) - np.array([x_robot, y_robot]))