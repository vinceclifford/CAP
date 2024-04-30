import pygame 
import sys 
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns 
from classes.robot import Robot
from classes.static_circle import Static_Circle
from classes.static_polygon import Static_Polygon
from engine_math import norm, calculate_total_force, calculate_potential_field_value_temperature 
from field_with_dijkstra import pathplanning_with_potential_field_and_dijkstra
from functools import partial
from environments.environment_1 import obstacles, agent, target 

SCREEN_WIDTH = 800
DELTA = 5
STEP_SIZE = 1
SCREEN_HEIGHT = int(SCREEN_WIDTH * 0.8)
WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (255, 0, 0)
PURPLE = (160, 32, 240)

pygame.init()  
pygame.display.set_caption("Gradient Descent")
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
screen.fill(WHITE)
clock = pygame.time.Clock()

def draw_robot(robot, color, radius): 
    pygame.draw.circle(screen, color, (int(robot.vektor[0]), int(robot.vektor[1])), radius)


def draw_obstacles(): 
    for entry in obstacles: 
        if isinstance(entry, Static_Circle): 
            pygame.draw.circle(screen, PURPLE, (int(entry.vektor[0]), int(entry.vektor[1])), 7)
        elif isinstance(entry, Static_Polygon): 
            pygame.draw.polygon(screen, PURPLE, entry.vertices, width=0)

if SCREEN_WIDTH <= 0: 
    exit("A non positive Screen width was entered")
    
draw_robot(agent, BLACK, 5)
draw_robot(target, RED, 10)
draw_obstacles()
pygame.display.flip()

def gradient_descent(robot, target, obstacle_set): 
    done = False
    iteration = 0 
    while not done: 
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:  
                done = True  
        if distance(robot.vektor, target.vektor) < DELTA: 
            break  
        total_force = calculate_total_force(robot,target, obstacle_set)
        distance = norm(total_force)
        normalized_total_force = (total_force[0] / distance), (total_force[1] / distance)
        robot.vektor = robot.vektor[0] + normalized_total_force[0] * STEP_SIZE, robot.vektor[1] + normalized_total_force[1] * STEP_SIZE
        draw_robot(robot, BLACK, 5)
        pygame.display.flip()
        clock.tick(120)


def visualization_heat_map(alpha, temp): 
    outer_array = []
    visualization_robot = Robot(0,0)
    max = -1
    position = None
    for x in range(SCREEN_WIDTH): 
        inner_array = []
        for y in range(SCREEN_HEIGHT): 
            visualization_robot.vektor = x,y
            value = calculate_potential_field_value_temperature(target, obstacles, alpha, temp, visualization_robot ) 
            if value == sys.float_info.max: 
                inner_array.append(sys.maxsize)
            else: 
                inner_array.append(value)
                if value > max: 
                    max = value 
        outer_array.append(inner_array)
        
    outer_array = np.transpose(outer_array)

    data = np.array(outer_array)
    sns.heatmap(data, cmap='viridis', vmax=max)
    plt.title('Heatmap of Robot')
    plt.xlabel('x coordinate')
    plt.ylabel('y coordinate')

    plt.show()
    

def visualizaion_3d_function(alpha, temp):
    curried = partial(calculate_potential_field_value_temperature, target, obstacles, alpha, temp)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(SCREEN_WIDTH, 0)

    x = np.arange(0, SCREEN_WIDTH , 1)
    y = np.arange(0, SCREEN_HEIGHT, 1 )
    X, Y = np.meshgrid(x,y)
    zs = np.array([curried(Robot(x,y)) for x,y in zip(np.ravel(X), np.ravel(Y))])
    print(zs)
    Z = zs.reshape(X.shape)
    surf = ax.plot_surface(X, Y, Z, cmap='viridis')
    
    colorbar = fig.colorbar(surf, ax=ax, shrink=0.7)
    colorbar.outline.set_visible(False)

    ax.set_xlabel('x coordinate')
    ax.set_ylabel('y coordinate')
    ax.set_zlabel('value of potential field')
    plt.title('Value function of Robot')

    plt.show()  
    
    
def heat_field_with_dijkstra(): 
    path_points = pathplanning_with_potential_field_and_dijkstra(agent, target, obstacles, 1, 1, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.lines(screen, BLACK, False, path_points, 2)
    pygame.display.flip()


def main(): 
    #visualizaion_3d_function(1,1)
    heat_field_with_dijkstra()
    visualization_heat_map(1,1)
    
    while True: 
        done = False
        iteration = 0 
        while not done: 
            for event in pygame.event.get():  
                if event.type == pygame.QUIT:  
                    done = True  

if __name__ == "__main__": 
    main() 