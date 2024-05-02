import math
import sys 
import copy 
import numpy as np
from scipy.optimize import minimize
from functools import partial 

SIGMA_ZERO = 100    
DELTA_FOR_ZERO = 1

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
        result = calculate_single_repulsion(robot, obstacle)
        if result == sys.float_info.max: 
            return sys.float_info.max
        sum_of_rep += result
    return sum_of_rep + calculate_attraction(robot, target) 


def calculate_potential_field_value_temperature(target, obstacles, alpha, temp, robot): 
    """Summary calulcate_potential_field_value_temperature(): Will return the potential field value given the robot and target. 

    Args:
        target (static_circle): Object of the goal
        obstacles (list): List of obstacles
        alpha (float): alpha for deterministic annealing 
        temp (float): temp for deterministic annealing
        robot (static_circle): Object of the robot 


    Returns:
        float: Value of potential field value given the robots position 
    """
    # The entire repulsive force an agent will receive is the sum of each indivdual repulsive force 
    sum_of_rep = 0 
    for obstacle in obstacles: 
        result = calculate_single_repulsion(robot, obstacle, alpha, temp)
        
        # If we receive an infeasible point that we can't return we don't add up the repulsive forces. This could lead to an overflow
        if result == sys.float_info.max: 
            return sys.float_info.max
        sum_of_rep += result
    return sum_of_rep + calculate_attraction(robot, target, alpha, temp) 


def distance(first_vektor, second_vektor): 
    return np.linalg.norm(np.array(first_vektor) - np.array(second_vektor))


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
    """Summary calculate_attraction(): Will return the attraction value given the robot and target. 

    Args:
        robot (static_circle): Object of the robot 
        target (static_circle): Object of the goal
        alpha (float): alpha for deterministic annealing 
        temp (float): temp for deterministic annealing

    Returns:
        float: Value of attraction force.
    """
    distance_val = distance(robot.vektor, target.vektor) / 100
    
    # We would divide by 0 in the potential field value function if the distance is 0. Therefore we calulate the value of the potential field 
    # value for a small delta. This will circumnavigate the problem with the division by zero. This small delta is set by a global parameter
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
    """Summary calculate_single_repulsion(): Will return the repulsion value given the robot and target. 

    Args:
        robot (static_circle): Object of the robot 
        target (static_circle): Object of the goal
        alpha (float): alpha for deterministic annealing 
        temp (float): temp for deterministic annealing

    Returns:
        float: Value of repulsive force.
    """
    distance_val = obstacle.distance(robot.vektor[0], robot.vektor[1])

    # We must guarantee that the robot does not hit the obstacle in a given radius, specified by the parameter obstacle.no_interference.
    # This will be achieved by giving that point an infinite amount of repulsion value. We will never chose to traverse this path. 
    if distance_val <= obstacle.no_interference: 
        return sys.float_info.max
    distance_by_alptemp = distance_val / (alpha * temp)
    
    # If it's a possible point we are allowed to traverse we input the distance into the function I came up with during research 
    difference = 50 * math.exp(-0.001 * 100/obstacle.distance_of_influence * (distance_by_alptemp ** 2))
    return difference * 0.5 * obstacle.attraction


def calculate_single_repulsion_force(robot, obstacle): 
    diff = distance(robot.vektor, obstacle.vektor)
    # If robot is far enough from obstacle it should not receive any force by it 
    if diff > obstacle.distance_of_influence: 
       return (0.0, 0.0)
    
    difference_of_vektors = (robot.vektor[0] - obstacle.vektor[0], robot.vektor[1] - obstacle.vektor[1])
    distance = distance(robot.vektor, obstacle.vektor)
    normalized_vektor = (difference_of_vektors[0] / distance, difference_of_vektors[1] / distance)
    coefficients = obstacle.attraction * (1/distance - 1/obstacle.distance_of_influence) * 1/(distance ** 2)
    return (normalized_vektor[0] * coefficients, normalized_vektor[1] * coefficients)