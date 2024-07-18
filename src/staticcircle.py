import numpy as np

class StaticCircle:
    def __init__(self, x, y, radius, attraction, no_interference=10) -> None:
        self.vector = (x, y)
        self.distance_of_influence = radius
        self.attraction = attraction
        self.no_interference = no_interference

    def distance(self, x_robot, y_robot):
        return np.linalg.norm(np.array([self.vector[0], self.vector[1]]) - np.array([x_robot, y_robot]))