import math
import sys 
import copy 
from scipy.optimize import minimize
from functools import partial 

SIGMA_ZERO = 100    
DELTA_FOR_ZERO = 1

def norm(vektor): 
    sum = 0 
    for entry in vektor: 
        sum += (entry ** 2)
    return math.sqrt(sum)


def preprocessing(target, obstacles, width, height): 
    temp = sys.maxsize
    alpha = 0.995
    tolerance = 3 
    minimization_point =  minimize_temp(target, obstacles, alpha, temp, width, height)
    
    while True: 
       temp *= alpha
       next_minimization_point = minimize_temp(target, obstacles, alpha, temp, width, height)
       if distance(minimization_point, next_minimization_point) < tolerance: 
           return minimization_point
       minimization_point = next_minimization_point        


def minimize_temp(target, obstacles, alpha, temp, width, height): 
    """We try to find the minimum of the potential field value with given cooling factor alpha and a specific temparature. This temperature will decrease steadily 
    We only search within the bounds of the coordinate system / the operating room which is represented as a coordinate system 
    minimze takes a function as well as a starting value. It minimizes the function which has one paramter. Hence, we need to curry the function that calculates the 
    entire potential field function. The curried function will only depend on the (x,y) values of the robot"""
    
    bounds = [(0, width), (0, height)]
    curried = partial(calculate_potential_field_value_temperature, target, obstacles, alpha, temp)
    #TODO Missing of the subtraction of TH 
    x_0 = [0,0]
    result = minimize(curried, x_0, bounds=bounds)  
    return result.x


def calculate_potential_field_value(robot, target, obstacles): 
    sum_of_rep = 0
    for obstacle in obstacles: 
        sum_of_rep += calculate_single_repulsion(robot, obstacle)
    return sum_of_rep + calculate_attraction(robot, target) 


def calculate_potential_field_value_temperature(target, obstacles, alpha, temp, robot): 
    sum_of_rep = 0 
    for obstacle in obstacles: 
        sum_of_rep += calculate_single_repulsion(robot, obstacle, alpha, temp)
    return sum_of_rep + calculate_attraction(robot, target, alpha, temp) 


def distance(first_vektor, second_vektor): 
    intermediate = first_vektor[0] - second_vektor[0], first_vektor[1] - second_vektor[1]
    return norm(intermediate)


def calculate_total_force(robot, target, obstacle_set): 
    sum_repulsion_force = (0, 0)
    for obstacle in obstacle_set: 
        single_force = calculate_single_repulsion_force(robot, obstacle)
        sum_repulsion_force = sum_repulsion_force[0] + single_force[0], sum_repulsion_force[1] + single_force[1]
    attraction_force = calculate_attraction_force(robot, target)
    result = sum_repulsion_force[0] + attraction_force[0], sum_repulsion_force[1] + attraction_force[1]
    print(f"{result[0]}, {result[1]}")
    return result


def calculate_attraction(robot, target, alpha=1, temp=1):
    distance_val = distance(robot.vektor, target.vektor) / 100
    if distance_val == 0: 
        robot_with_delta = copy.deepcopy(robot)
        robot_with_delta.vektor = (robot.vektor[0] + DELTA_FOR_ZERO, robot.vektor[1] + DELTA_FOR_ZERO)
        return calculate_attraction(robot_with_delta, target, alpha, temp)
    if (distance_val/temp) <= SIGMA_ZERO: 
        return 0.5 * target.attraction * (distance_val ** 2) / (alpha * temp)
    return (target.attraction * distance(robot.vektor, target.vektor) / 100 * SIGMA_ZERO - 0.5 * target.attraction * SIGMA_ZERO * SIGMA_ZERO) / (alpha * temp)


def calculate_attraction_force(robot, target): 
    """ If distance between current position and target is less than SIGMA_ZERO, the farther the distance is, the greater the attractive 
    force of the robot will be. It's exactly the opposit if the distance is greater than SIGMA_ZERO. This prevents problems of 
    collision with obstacles caused by too much attractive force."""
    
    x_result = -target.attraction * (robot.vektor[0] - target.vektor[0])
    y_result = -target.attraction * (robot.vektor[1] - target.vektor[1])
    dividend = distance(robot.vektor, target.vektor)

    if dividend <= SIGMA_ZERO: 
        return (x_result, y_result)
    
    return ((x_result * SIGMA_ZERO) / dividend, (y_result * SIGMA_ZERO) / dividend)


def calculate_single_repulsion(robot, obstacle, alpha=1, temp=1): 
    distance_val = obstacle.distance(robot.vektor[0], robot.vektor[1])
    #if (distance_val / temp) > obstacle.radius_of_influence: 
     #   return 0.0
    if distance_val == 0: 
        robot_with_delta = copy.deepcopy(robot)
        robot_with_delta.vektor = (robot.vektor[0] + DELTA_FOR_ZERO, robot.vektor[1] + DELTA_FOR_ZERO)
        return calculate_single_repulsion(robot_with_delta, obstacle, alpha, temp)
    distance_by_alptemp = distance_val / (alpha * temp)
    
    difference = 50 * math.exp(-0.001 * (distance_by_alptemp ** 2))
    return difference * 0.5 * obstacle.attraction


def calculate_single_repulsion_force(robot, obstacle): 
    diff = distance(robot.vektor, obstacle.vektor)
    # If robot is far enough from obstacle it should not receive any force by it 
    if diff > obstacle.radius_of_influence: 
       return (0.0, 0.0)
    
    difference_of_vektors = (robot.vektor[0] - obstacle.vektor[0], robot.vektor[1] - obstacle.vektor[1])
    distance = distance(robot.vektor, obstacle.vektor)
    normalized_vektor = (difference_of_vektors[0] / distance, difference_of_vektors[1] / distance)
    coefficients = obstacle.attraction * (1/distance - 1/obstacle.radius_of_influence) * 1/(distance ** 2)
    return (normalized_vektor[0] * coefficients, normalized_vektor[1] * coefficients)