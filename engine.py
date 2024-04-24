import pygame 
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns 
from classes.robot import Robot
from classes.static_objects import Static_Object
from engine_math import norm, calculate_total_force, calculate_potential_field_value, calculate_potential_field_value_temperature 
from field_with_dijkstra import pathplanning_with_potential_field_and_dijkstra

SCREEN_WIDTH = 800
DELTA = 5
STEP_SIZE = 1
SCREEN_HEIGHT = int(SCREEN_WIDTH * 0.8)
WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (255, 0, 0)
PURPLE = (160, 32, 240)

obstacles = set() 
target = Static_Object(400, 400, 30, 3)
agent = Robot(45, 45)
obstacle_1 = Static_Object(200, 200, 80, 3)
obstacle_2 = Static_Object(120, 140, 30, 3)
obstacle_3 = Static_Object(170, 120, 50, 2)
obstacle_4 = Static_Object(300, 300, 70, 3)
obstacle_5 = Static_Object(270, 300, 40, 2.5)
obstacle_6 = Static_Object(400, 375, 75, 3)
obstacle_7 = Static_Object(100, 277, 50, 3)
obstacle_8 = Static_Object(420, 430, 40, 3)
obstacle_9 = Static_Object(400, 60, 70, 4)
obstacles.add(obstacle_1)
obstacles.add(obstacle_2)
obstacles.add(obstacle_3)
obstacles.add(obstacle_4)
obstacles.add(obstacle_5)
obstacles.add(obstacle_6)
obstacles.add(obstacle_7)
obstacles.add(obstacle_8)
obstacles.add(obstacle_9)

pygame.init()  
pygame.display.set_caption("Gradient Descent")
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
screen.fill(WHITE)
clock = pygame.time.Clock()

def draw_robot(robot, color, radius): 
    pygame.draw.circle(screen, color, (int(robot.vektor[0]), int(robot.vektor[1])), radius)

def draw_obstacles(): 
    for entry in obstacles: 
        pygame.draw.circle(screen, PURPLE, (int(entry.vektor[0]), int(entry.vektor[1])), 7)

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

def heat_map_visualization(): 
    outer_array = []
    visualization_robot = Robot(0,0)
    max = -1
    position = None
    for x in range(SCREEN_WIDTH): 
        inner_array = []
        for y in range(SCREEN_HEIGHT): 
            visualization_robot.vektor = x,y
            value = calculate_potential_field_value_temperature(target, obstacles, 1, 1, visualization_robot ) 
            inner_array.append(value)
        outer_array.append(inner_array)
        
    outer_array = np.transpose(outer_array)

    print(max)
    print(position)

    data = np.array(outer_array)
    sns.heatmap(data, cmap='viridis')
    plt.title('Heatmap of Robot')
    plt.xlabel('X Axis Label')
    plt.ylabel('Y Axis Label')

    plt.show()
    
def heat_field_with_dijkstra(): 
    path_points = pathplanning_with_potential_field_and_dijkstra(agent, target, obstacles, 1, 1, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.lines(screen, BLACK, False, path_points, 2)
    pygame.display.flip()

def main(): 
    heat_field_with_dijkstra()
    heat_map_visualization()
    while True: 
        done = False
        iteration = 0 
        while not done: 
            for event in pygame.event.get():  
                if event.type == pygame.QUIT:  
                    done = True  

if __name__ == "__main__": 
    main() 