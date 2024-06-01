import pygame 
import sys 
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns 
from classes.robot import Robot
from classes.static_circle import Static_Circle
from classes.static_polygon import Static_Polygon
from math_engine import calculate_total_force, calculate_potential_field_value_temperature, distance
from field_with_dijkstra import pathplanning_with_potential_field_and_dijkstra, dijkstra
from math_tensor import create_potential_field_value_tensor
from dijkstra import dijkstra_on_nparray_with_dictionary_without_detach
from functools import partial           
from environments.environment_1 import obstacles, agent, target 
import time
import torch
from math_tensor import fill_infinite_polygon_repulsive_field_value

SCREEN_WIDTH = 800
DELTA = 5
STEP_SIZE = 1
SCREEN_HEIGHT = 640
WHITE = (255,255,255)
BLACK = (0,0,0)
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
    pygame.draw.circle(screen, color, (int(robot.vektor[0]), int(robot.vektor[1])), radius)


def draw_obstacles(screen, obstacles, color, radius): 
    """Summary of function draw_obstacles(): Will draw all obstacles on screen for visualization purposes. 
    Depending on whether the obstacle is a polygon or circle a different obstacle will be rendered

    Args:
        Same as in the function draw_robot apart from robot. obstacles is of type list. 
    """
    
    for entry in obstacles: 
        if isinstance(entry, Static_Circle): 
            pygame.draw.circle(screen, color, (int(entry.vektor[0]), int(entry.vektor[1])), radius)
        elif isinstance(entry, Static_Polygon): 
            pygame.draw.polygon(screen, PURPLE, entry.vertices, width=0)


def gradient_descent(clock, robot, target, obstacle_set): 
    done = False
    iteration = 0 
    while not done: 
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:  
                done = True  
        if distance(robot.vektor, target.vektor) < DELTA: 
            break  
        total_force = calculate_total_force(robot,target, obstacle_set)
        d = distance(total_force, (0,0))
        normalized_total_force = (total_force[0] / d), (total_force[1] / d)
        robot.vektor = robot.vektor[0] + normalized_total_force[0] * STEP_SIZE, robot.vektor[1] + normalized_total_force[1] * STEP_SIZE
        draw_robot(robot, BLACK, 5)
        pygame.display.flip()
        clock.tick(120)


def visualization_heat_map(alpha, temp): 
    """Summary of visualization_heat_map(): Will display a heat map of the potential field value function 

    Args:
        alpha (int): alpha for deterministic annealing 
        temp (int): temp for deterministic annealing
    """
    outer_array = []
    visualization_robot = Robot(0,0)
    max = -1
    for x in range(SCREEN_WIDTH): 
        inner_array = []
        for y in range(SCREEN_HEIGHT): 
            visualization_robot.vektor = x,y
            value = calculate_potential_field_value_temperature(target, obstacles, alpha, temp, visualization_robot) 
            
            # Cut of the maximum value. This is due to the fact that otherwise a scaled heat map would be unusuable. You would only see 
            # the difference between the max and non-max values. Nothing in between. 
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
    print(max)
    sns.heatmap(np_array, cmap='viridis', vmax=max)
    plt.title('Heatmap of Robot')
    plt.xlabel('x coordinate')
    plt.ylabel('y coordinate')

    plt.show()
    

def visualizaion_3d_function(alpha, temp):
    """Summary of visualizaion_3d_function: Will create a 3D function. This visualized function will receive two inputs, the coordinates of 
    the robot, and will calculate the potential field value. 

    Args:
        alpha (int): alpha for deterministic annealing 
        temp (int): temp for deterministic annealing
    """
    curried = partial(calculate_potential_field_value_temperature, target, obstacles, alpha, temp)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(SCREEN_WIDTH, 0)

    x = np.arange(0, SCREEN_WIDTH , 1)
    y = np.arange(0, SCREEN_HEIGHT, 1 )
    X, Y = np.meshgrid(x,y)
    zs = np.array([curried(Robot(x,y)) for x,y in zip(np.ravel(X), np.ravel(Y))])
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
        if obstacle.vektor[0] < 0 or obstacle.vektor[0] > SCREEN_WIDTH or obstacle.vektor[1] < 0 or obstacle.vektor[1] > SCREEN_HEIGHT: 
            exit("One of the obstacles is not in the bounds of our environment!")


def main(): 
    
    check_validity_of_obstacles(obstacles)
    pygame.init()  
    pygame.display.set_caption("Environment")
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    screen.fill(GREY)
    clock = pygame.time.Clock()
    
    if SCREEN_WIDTH <= 0: 
        exit("A non positive Screen width was entered")
    
    draw_robot(screen, agent, BLACK, 5)
    draw_robot(screen, target, RED, 10)
    draw_obstacles(screen, obstacles, PURPLE, 7)
    pygame.display.flip()
    
    tensor = create_potential_field_value_tensor(obstacles, target, SCREEN_WIDTH, SCREEN_HEIGHT)
    starting_time = time.time()
    path, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor, agent.vektor, target.vektor)
    #path, _ = pathplanning_with_potential_field_and_dijkstra(agent, target, obstacles, 800, 640)
    finishing_time = time.time()
    #visualizaion_3d_function(1,1)
    visualizing_dijkstra(screen, path)
    #visualization_heat_map_tensor(tensor)
    
    diff = finishing_time - starting_time
    print(f"Computational time at {diff}")
    while True: 
        done = False
        iteration = 0 
        while not done: 
            for event in pygame.event.get():  
                if event.type == pygame.QUIT:  
                    done = True



def main_2():
    obstacle_1 = Static_Polygon([(100, 100), (150, 150), (200, 100), (150, 50)], 5, 3, no_interference=2)
    obstacle_2 = Static_Polygon([(260, 10), (260, 300), (300, 20)], 5, 3, no_interference=2)

    tensor = torch.arange(0, 800 + 1, dtype=torch.float32)
    tensor = tensor.unsqueeze(0).repeat(640 + 1, 1)
    tensor = fill_infinite_polygon_repulsive_field_value([obstacle_2], tensor)
    visualization_heat_map_tensor(tensor)


if __name__ == "__main__": 
    main_2()