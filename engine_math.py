import math

SIGMA_ZERO = 0.1

def norm(vektor): 
    sum = 0 
    for entry in vektor: 
        sum += (entry ** 2)
    return math.sqrt(sum)

def norm_subtraction(first_vektor, second_vektor): 
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

def calculate_attraction(robot, target): 
    if norm_subtraction(robot.vektor, target.vektor) <= SIGMA_ZERO: 
        return 0.5 * target.attraction * (norm_subtraction(robot.vektor, target.vektor)**2)
    return target.attraction * norm_subtraction(robot.vektor, target.vektor) * SIGMA_ZERO - 0.5 * target.attraction * SIGMA_ZERO * SIGMA_ZERO

"""
If distance between current position and target is less than SIGMA_ZERO, the farther the distance is, the greater the attractive 
force of the robot will be. It's exactly the opposit if the distance is greater than SIGMA_ZERO. This prevents problems of 
collision with obstacles caused by too much attractive force. 
"""
def calculate_attraction_force(robot, target): 
    x_result = -target.attraction * (robot.vektor[0] - target.vektor[0])
    y_result = -target.attraction * (robot.vektor[1] - target.vektor[1])
    dividend = norm_subtraction(robot.vektor, target.vektor)

    if dividend <= SIGMA_ZERO: 
        return (x_result, y_result)
    
    return ((x_result * SIGMA_ZERO) / dividend, (y_result * SIGMA_ZERO) / dividend)

def calculate_single_repulsion(robot, obstacle): 
    if norm_subtraction(robot.vektor, obstacle.vektor) > obstacle.radius_of_influence: 
        return 0.0
    return 0.5 * obstacle.attraction * ((1.0/norm_subtraction(robot.vektor, obstacle.vektor)) - 1.0/obstacle.radius_of_influence) ** 2 

"""
If robot is far enough from obstacle it should not receive any force by it 
"""
def calculate_single_repulsion_force(robot, obstacle): 
    diff = norm_subtraction(robot.vektor, obstacle.vektor)
    #if diff > obstacle.radius_of_influence: 
        #print(f"The distance between the two objects is {diff} with robot position {robot.vektor[0]}, {robot.vektor[1]} and the radius of influence should be {obstacle.radius_of_influence}")
       # return (0.0, 0.0)
    
    difference_of_vektors = (robot.vektor[0] - obstacle.vektor[0], robot.vektor[1] - obstacle.vektor[1])
    distance = norm_subtraction(robot.vektor, obstacle.vektor)
    normalized_vektor = (difference_of_vektors[0] / distance, difference_of_vektors[1] / distance)
    coefficients = obstacle.attraction * (1/distance - 1/obstacle.radius_of_influence) * 1/(distance ** 2)
    return (normalized_vektor[0] * coefficients, normalized_vektor[1] * coefficients)

