import numpy as np
import math


def calculate_average_velocity(coords):
    if len(coords) < 2:
        return 0

    total_distance = 0

    for i in range(1, len(coords)):
        x1, y1 = coords[i - 1]
        x2, y2 = coords[i]
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        total_distance += distance

    total_time = (len(coords) - 1)
    average_velocity = total_distance / total_time

    return average_velocity


class StaticCircle:
    def __init__(self, x, y, radius, attraction, no_interference=10) -> None:
        self.vector = (x, y)
        self.distance_of_influence = radius
        self.attraction = attraction
        self.no_interference = no_interference

    def distance(self, x_robot, y_robot):
        return np.linalg.norm(np.array([self.vector[0], self.vector[1]]) - np.array([x_robot, y_robot]))

    def collides_with_point(self, coordinates, radius):
        return self.distance(coordinates[0], coordinates[1]) - radius - self.no_interference <= 0

    def collides_even_with_waiting(self, coordinates, radius, velocity_history):
        # This function return a tuple of boolean values. The first value indicates whether we collide with an obstacle.
        # The second one tells whether we should calculate a new path or should wait

        average_velocity = calculate_average_velocity(velocity_history)
        if not self.collides_with_point(coordinates, radius):
            return False, False

        print(f"The radius of both obstacle and robot is {radius} / {self.no_interference}")
        print(f"The distance to the center of the object is {self.distance(coordinates[0], coordinates[1])}")
        distance_to_clearance = radius + self.no_interference - self.distance(coordinates[0], coordinates[1])
        print(f"Distance to clear: {distance_to_clearance}")

        print(f"Estimation of amount of frames needed {distance_to_clearance / average_velocity}")
        if (distance_to_clearance / average_velocity) < 10:
            return True, False

        return True, True
