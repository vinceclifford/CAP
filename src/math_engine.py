import math
import sys
import copy
import numpy as np

SIGMA_ZERO = 50
DELTA_FOR_ZERO = 1


def calculate_potential_field_value(robot, target, obstacles):
    """Summary calculate_potential_field_value(): Will return the potential field value given the robot and target.

    Args:
        target (static_circle): Object of the goal.
        obstacles (list): List of obstacles.
        robot (static_circle): Object of the robot.

    Returns:
        float: Value of potential field value given the robots position.
    """

    sum_of_rep = 0
    for obstacle in obstacles:
        result = calculate_single_repulsion(robot, obstacle)
        if result == sys.float_info.max:
            return sys.float_info.max
        sum_of_rep += result

    return sum_of_rep + calculate_attraction(robot, target)


def calculate_potential_field_value_temperature(target, obstacles, alpha, temp, robot):
    """Summary calculate_potential_field_value_temperature(): Will return the potential field value given the robot
    and target.

    Args:
        target (static_circle): Object of the goal.
        obstacles (list): List of obstacles.
        alpha (float): alpha for deterministic annealing.
        temp (float): temp for deterministic annealing.
        robot (static_circle): Object of the robot.

    Returns:
        float: Value of potential field value given the robots position.
    """

    # The entire repulsive force an agent will receive is the sum of each individual repulsive force
    sum_of_rep = 0
    for obstacle in obstacles:
        result = calculate_single_repulsion(robot, obstacle, alpha, temp)

        # If we receive an infeasible point that we can't return we don't add up the repulsive forces. This could
        # lead to an overflow
        if result == sys.float_info.max:
            return sys.float_info.max
        sum_of_rep += result
    return sum_of_rep + calculate_attraction(robot, target, alpha, temp)


def distance(first_vector, second_vector):
    """
    Summary distance(): Will return the distance between two vectors. It's a wrapper function.

    Args:
        first_vector (int, int): First vector.
        second_vector (int, int): Second vector.

    Returns:
        Will return the distance between two vectors.
    """

    return np.linalg.norm(np.array(first_vector) - np.array(second_vector))


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
    distance_val = distance(robot.vector, target.vector) / 100

    # We would divide by 0 in the potential field value function if the distance is 0. Therefore, we calculate the
    # value of the potential field value for a small delta. This will circumnavigate the problem with the division by
    # zero. This small delta is set by a global parameter
    if distance_val == 0:
        robot_with_delta = copy.deepcopy(robot)
        robot_with_delta.vector = (robot.vector[0] + DELTA_FOR_ZERO, robot.vector[1] + DELTA_FOR_ZERO)
        return calculate_attraction(robot_with_delta, target, alpha, temp)
    if (distance_val / temp) <= SIGMA_ZERO:
        return 0.5 * target.attraction * (distance_val ** 2) / (alpha * temp)
    return (target.attraction * distance(robot.vector, target.vector) / 100 * SIGMA_ZERO - 0.5 * target.attraction *
            SIGMA_ZERO * SIGMA_ZERO) / (alpha * temp)


# Does not work at the moment due to code rewrite and tensor math. We most likely will never use this code again.
def calculate_single_repulsion(robot, obstacle, alpha=1, temp=1):
    """Summary calculate_single_repulsion(): Will return the repulsion value given the robot and target. 

    Args:
        obstacle (StaticCircle or StaticPolygon): Object of the obstacle.:
        robot (static_circle): Object of the robot.
        alpha (float): alpha for deterministic annealing.
        temp (float): temp for deterministic annealing.

    Returns:
        float: Value of repulsive force.
    """

    distance_val = obstacle.distance(robot.vector[0], robot.vector[1])

    # We must guarantee that the robot does not hit the obstacle in a given radius, specified by the parameter
    # obstacle.no_interference. This will be achieved by giving that point an infinite amount of repulsion value. We
    # will never choose to traverse this path.
    if distance_val <= obstacle.no_interference:
        return sys.float_info.max
    distance_by_alphtemp = distance_val / (alpha * temp)

    # If it's a possible point we are allowed to traverse we input the distance into the function I came up with
    # during research
    difference = 50 * math.exp(-0.001 * 100 / obstacle.distance_of_influence * (distance_by_alphtemp ** 2))
    return difference * 0.5 * obstacle.attraction
