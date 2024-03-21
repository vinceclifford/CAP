import pygame 
from robot import Robot
from static_objects import Static_Object
from engine_math import norm, norm_subtraction, calculate_total_force

SCREEN_WIDTH = 800
DELTA = 5
STEP_SIZE = 1
SCREEN_HEIGHT = int(SCREEN_WIDTH * 0.8)
WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (255, 0, 0)
PURPLE = (160, 32, 240)

obstacles = set() 
target = Static_Object(200, 200, 200, 0.05)
agent = Robot(20, 20)
obstacle_1 = Static_Object(70, 50, 40,  80)
obstacle_2 = Static_Object(120, 140, 80, 80)
obstacles.add(obstacle_1)
obstacles.add(obstacle_2)

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
        if norm_subtraction(robot.vektor, target.vektor) < DELTA: 
            break  
        total_force = calculate_total_force(robot,target, obstacle_set)
        distance = norm(total_force)
        normalized_total_force = (total_force[0] / distance), (total_force[1] / distance)
        robot.vektor = robot.vektor[0] + normalized_total_force[0] * STEP_SIZE, robot.vektor[1] + normalized_total_force[1] * STEP_SIZE
        draw_robot(robot, BLACK, 5)
        pygame.display.flip()
        clock.tick(30)

gradient_descent(agent, target, obstacles)