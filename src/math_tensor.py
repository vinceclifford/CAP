import torch
from classes.static_circle import Static_Circle
import matplotlib.pyplot as plt
import seaborn as sns


C_DOUBLE_PRIME = -0.001
C = 100

def create_potential_field_value_tensor(obstacles, target, width, height, alpha=1, temp=1): 
    """Summary of function create_tensor_repulsive_force(): This function will calculate the potential field value for the entire obstacle and store the value of each pixel (x,y) in a tensor. Calulating the potential field value function will be done through Pytorch. This will automatically result in parallel execution  
       
    Args:
        obstacle (list): list of obstacles in given environment 
        target (static_circle): object of goal 
        width (int): width of the environment 
        height (int): height of the environment 
        alpha (float): alpha for deterministic annealing. Defaults to 1.
        temp (float): temp for deterministic annealing. Defaults to 1.


    Returns:
        2D-tensor: Will return a 2D tensor of the potential field value 
    """
    
    basic_tensor_row = torch.arange(1, width + 1, dtype=torch.float32)  
    basic_tensor_column = torch.arange(1, height + 1, dtype=torch.float32)
    
    first_layer_tensor = basic_tensor_row.unsqueeze(0).repeat(height, 1)
    second_layer_tensor = basic_tensor_column.unsqueeze(0).repeat(width, 1).t()
    
    # 3D tensor with the dimensions (width, height,2) with x and y coordinates 
    position_tensor = torch.stack((first_layer_tensor, second_layer_tensor), dim=2)
    
    # 2D tensor with the x and y coordinate of each obstacle in it 
    obstacle_position_tensor = torch.empty(len(obstacles), 2)
    
    # 1D tensor which will only store the attraction values of each individual obstacle. The order of the obstacles in  obstacle_position_tensor and obstacle_attraction_tensor must be the same. 
    obstacle_distance_of_influence_tensor = torch.empty(len(obstacles))
    osbtacle_attraction_tensor = torch.empty(len(obstacles))
     
    for index, obstacle in enumerate(obstacles): 
        obstacle_position_tensor[index] = torch.tensor(obstacle.vektor).to(obstacle_position_tensor.dtype)
        obstacle_distance_of_influence_tensor[index] = torch.tensor(obstacle.distance_of_influence).to(obstacle_distance_of_influence_tensor.dtype)
        osbtacle_attraction_tensor = torch.tensor(obstacle.attraction).to(obstacle_distance_of_influence_tensor.dtype)

    # General formula for repulsive formula: c' * math.exp(c'' * c/obstacle.distance_of_influence * (distance_by_alptemp ** 2)) Here we calculate everything except for the distance_by_alptemp ** 2 in the exponent 
    obstacle_distance_of_influence_tensor = (C_DOUBLE_PRIME * C) / obstacle_distance_of_influence_tensor
    
    position_tensor.unsqueeze_(2)
    obstacle_position_tensor.unsqueeze_(0).unsqueeze_(0)
    obstacle_distance_of_influence_tensor.unsqueeze_(0).unsqueeze_(0)
    osbtacle_attraction_tensor.unsqueeze_(0).unsqueeze_(0)

    # After adjusting the dimensions we are now able to perform operations to determine the euclidean distance to all obstacle. The distance of each position (x,y) to a respecive object j will be stored in the 4th dimenstion 
    #print(position_tensor.size())
    #print(obstacle_position_tensor.size())
    difference = position_tensor - obstacle_position_tensor
    squared = difference ** 2 
    
    # Collapsing the tensor in the 4th dimension. For every object j only the euclidean distance from point (x,y) to it will be stored in the cell (x,y,j)
    sum = torch.sum(squared, dim=3)
    sqrt = torch.sqrt(sum)
    
    alpha_temp = sqrt / (alpha * temp)
    squared_again = alpha_temp ** 2   
    
    
    # Cellwise multiplication to adjust for attraction_forces. Afterwards we will collapse the tensor into two dimension 
    multiplied = squared_again * obstacle_distance_of_influence_tensor
    repulsion_tensor = 50 * torch.exp(multiplied)
    repulsion_tensor = torch.sum(0.5 * repulsion_tensor * osbtacle_attraction_tensor, dim=2)
    #print(repulsion_tensor)

    
    # We set up tensors to determine the distance between all points in the environment and the target such that we can determine the attraction force for each pixel 
    attraction_tensor = torch.stack((first_layer_tensor, second_layer_tensor), dim=2)
    target_tensor = torch.tensor(target.vektor)
    
    # Adjust the dimesionality of target_tensor such that we can use the appropiate broadcasting functionalities 
    target_tensor.unsqueeze_(0)
    attraction_tensor = torch.sqrt(torch.sum((attraction_tensor - target_tensor) ** 2, dim=2)) / 100 
    
    # Code for the formula given by calculate_attraction in engine_math.py: return 0.5 * target.attraction * (distance_val ** 2) / (alpha * temp)
    coefficient = 0.5 * target.attraction / (alpha * temp)
    attraction_tensor = coefficient * (attraction_tensor ** 2)
    
    result = attraction_tensor + repulsion_tensor
    max_val = torch.max(result).item()
    
    # Still have to guarantee that we don't hit an obstacle it its no_interference zone. Therefore, we need to set the potential field value in these anges to infinity. Atm I don't have a more efficient idea
    for obstacle in obstacles: 
        for x in range(obstacle.vektor[0] - obstacle.no_interference, obstacle.vektor[0] + obstacle.no_interference + 1): 
            for y in range(obstacle.vektor[1] - obstacle.no_interference, obstacle.vektor[1] + obstacle.no_interference + 1): 
                diff = (torch.tensor(obstacle.vektor) - torch.tensor((x,y))).to(dtype=torch.float)
                if  torch.linalg.norm(diff).item() <= obstacle.no_interference: 
                    result[y, x] = torch.finfo(torch.float32).max
    
    return result