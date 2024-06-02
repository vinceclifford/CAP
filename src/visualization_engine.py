import pygame
import sys
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from classes.robot import Robot
from classes.staticcircle import StaticCircle
from classes.staticpolygon import StaticPolygon
from math_engine import calculate_potential_field_value_temperature
from functools import partial
import torch

SCREEN_WIDTH = 800
DELTA = 5
STEP_SIZE = 1
SCREEN_HEIGHT = 640
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
PURPLE = (160, 32, 240)
GREY = (230, 230, 230)


def draw_robot(screen, robot, color, radius):
    """Summary of function draw_robot(): Will draw robot on screen for visualization purposes

    Args:
        screen (pygame.surface.Surface): surface on which the trajectory is displayed
        robot (static_circle):  Object of the robot 
        color ((int, int, int)): color in which the robot should be visualized
        radius (int): radius of the displayed robot 
    """
    pygame.draw.circle(screen, color, (int(robot.vector[0]), int(robot.vector[1])), radius)


def draw_obstacles(screen, obstacles, color):
    """Summary of function draw_obstacles(): Will draw all obstacles on screen for visualization purposes. 
    Depending on whether the obstacle is a polygon or circle a different obstacle will be rendered

    Args:
        Same as in the function draw_robot apart from robot. obstacles is of type list. 
    """

    for entry in obstacles:
        if isinstance(entry, StaticCircle):
            pygame.draw.circle(screen, color, (int(entry.vector[0]), int(entry.vector[1])), entry.no_interference)
        elif isinstance(entry, StaticPolygon):
            if len(entry.vertices) == 2:
                pygame.draw.line(screen, PURPLE, entry.vertices[0], entry.vertices[1], width=2)
            else:
                pygame.draw.polygon(screen, PURPLE, entry.vertices, width=0)


def visualization_heat_map(alpha, temp, target, obstacles):
    """Summary of visualization_heat_map(): Will display a heat map of the potential field value function 

    Args:
        alpha (int): alpha for deterministic annealing 
        temp (int): temp for deterministic annealing
    """
    outer_array = []
    visualization_robot = Robot(0, 0)
    max = -1
    for x in range(SCREEN_WIDTH):
        inner_array = []
        for y in range(SCREEN_HEIGHT):
            visualization_robot.vector = x, y
            value = calculate_potential_field_value_temperature(target, obstacles, alpha, temp, visualization_robot)

            # Cut of the maximum value. This is due to the fact that otherwise a scaled heat map would be unusuable.
            # You would only see the difference between the max and non-max values. Nothing in between.
            if value == sys.float_info.max:
                inner_array.append(sys.maxsize)
            else:
                inner_array.append(value)
                if value > max:
                    max = value

        outer_array.append(inner_array)

    # Else the heat map is transposed.    
    outer_array = np.transpose(outer_array)

    data = np.array(outer_array)
    sns.heatmap(data, cmap='viridis', vmax=max)
    plt.title('Heatmap of Robot')
    plt.xlabel('x coordinate')
    plt.ylabel('y coordinate')

    plt.show()


def visualization_heat_map_tensor(tensor, alpha=1, temp=1):
    """Summary of visualization_heat_map(): Will display a heat map of the potential field value function 

    Args:
        alpha (int): alpha for deterministic annealing 
        temp (int): temp for deterministic annealing
    """
    np_array = tensor.numpy()

    max = np.max(np_array[np.logical_and(np.isfinite(np_array), np_array < torch.finfo(torch.float32).max)])
    sns.heatmap(np_array, cmap='viridis', vmax=max)
    plt.title('Heatmap of Robot')
    plt.xlabel('x coordinate')
    plt.ylabel('y coordinate')

    plt.show()


def visualizaion_3d_function(alpha, temp, target, obstacles):
    """Summary of visualizaion_3d_function: Will create a 3D function. This visualized function will receive two
    inputs, the coordinates of the robot, and will calculate the potential field value.

    Args:
        alpha (int): alpha for deterministic annealing 
        temp (int): temp for deterministic annealing
    """
    curried = partial(calculate_potential_field_value_temperature, target, obstacles, alpha, temp)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(SCREEN_WIDTH, 0)

    x = np.arange(0, SCREEN_WIDTH, 1)
    y = np.arange(0, SCREEN_HEIGHT, 1)
    X, Y = np.meshgrid(x, y)
    zs = np.array([curried(Robot(x, y)) for x, y in zip(np.ravel(X), np.ravel(Y))])
    Z = zs.reshape(X.shape)
    surf = ax.plot_surface(X, Y, Z, cmap='viridis', vmax=140)

    colorbar = fig.colorbar(surf, ax=ax, shrink=0.7)
    colorbar.outline.set_visible(False)

    ax.set_xlabel('x coordinate')
    ax.set_ylabel('y coordinate')
    ax.set_zlabel('value of potential field')
    plt.title('Value function of Robot')

    plt.show()


def visualizing_dijkstra(screen, path_points):
    """Summary of visualizing dijkstra(): After receiving trajectory path of dijkstra algorithm we display it.

    Args:
        screen (pygame.surface.Surface): surface on which the trajectory is displayed
    """
    print("Visualizing...")
    pygame.draw.lines(screen, BLACK, False, path_points, 2)
    pygame.display.flip()


def check_validity_of_obstacles(obstacles):
    for obstacle in obstacles:
        if isinstance(obstacles, StaticCircle):
            if obstacle.vector[0] < 0 or obstacle.vector[0] > SCREEN_WIDTH or obstacle.vector[1] < 0 or obstacle.vector[1] > SCREEN_HEIGHT:
                exit("One of the circle obstacles is not in the bounds of our environment!")
            else:
                pass
        elif isinstance(obstacle, StaticPolygon):
            looking_for_duplicate = set()
            for vertex in obstacle.vertices:
                if vertex in looking_for_duplicate:
                    exit("One of the polygons has got a duplicate vertex in the vertex list. This is not allowed!")
                if vertex[0] < 0 or vertex[0] > SCREEN_WIDTH or vertex[1] < 0 or vertex[1] >SCREEN_HEIGHT:
                    exit("One of the polygon vertices is not in the bounds of our environment!")
                looking_for_duplicate.add(vertex)
