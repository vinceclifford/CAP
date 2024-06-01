import torch
from classes.static_polygon import Static_Polygon
import time

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

    basic_tensor_row = torch.arange(0, width + 1, dtype=torch.float32)
    basic_tensor_column = torch.arange(0, height + 1, dtype=torch.float32)

    first_layer_tensor = basic_tensor_row.unsqueeze(0).repeat(height + 1, 1)
    second_layer_tensor = basic_tensor_column.unsqueeze(0).repeat(width + 1, 1).t()

    # 3D tensor with the dimensions (width, height,2) with x and y coordinates 
    position_tensor = torch.stack((first_layer_tensor, second_layer_tensor), dim=2)

    # 2D tensor with the x and y coordinate of each obstacle in it 
    obstacle_position_tensor = torch.empty(len(obstacles), 2)

    # 1D tensor which will only store the attraction values of each individual obstacle. The order of the obstacles
    # in  obstacle_position_tensor and obstacle_attraction_tensor must be the same.
    obstacle_distance_of_influence_tensor = torch.empty(len(obstacles))
    obstacle_attraction_tensor = torch.empty(len(obstacles))

    for index, obstacle in enumerate(obstacles):
        obstacle_position_tensor[index] = torch.tensor(obstacle.vektor).to(obstacle_position_tensor.dtype)
        obstacle_distance_of_influence_tensor[index] = torch.tensor(obstacle.distance_of_influence).to(
            obstacle_distance_of_influence_tensor.dtype)
        obstacle_attraction_tensor = torch.tensor(obstacle.attraction).to(obstacle_distance_of_influence_tensor.dtype)

    # General formula for repulsive formula: c' * math.exp(c'' * c/obstacle.distance_of_influence * (
    # distance_by_alptemp ** 2)) Here we calculate everything except for the distance_by_alptemp ** 2 in the exponent
    obstacle_distance_of_influence_tensor = (C_DOUBLE_PRIME * C) / obstacle_distance_of_influence_tensor

    position_tensor.unsqueeze_(2)
    obstacle_position_tensor.unsqueeze_(0).unsqueeze_(0)
    obstacle_distance_of_influence_tensor.unsqueeze_(0).unsqueeze_(0)
    obstacle_attraction_tensor.unsqueeze_(0).unsqueeze_(0)

    # After adjusting the dimensions we are now able to perform operations to determine the euclidean distance to all
    # obstacle. The distance of each position (x,y) to a respecive object j will be stored in the 4th dimenstion
    # print(position_tensor.size()) print(obstacle_position_tensor.size())
    difference = position_tensor - obstacle_position_tensor
    print(difference.size())
    squared = difference ** 2

    # Collapsing the tensor in the 4th dimension. For every object j only the euclidean distance from point (x,
    # y) to it will be stored in the cell (x,y,j)
    sum = torch.sum(squared, dim=3)
    sqrt = torch.sqrt(sum)

    alpha_temp = sqrt / (alpha * temp)
    squared_again = alpha_temp ** 2

    # Cellwise multiplication to adjust for attraction_forces. Afterwards we will collapse the tensor into two dimension 
    multiplied = squared_again * obstacle_distance_of_influence_tensor
    repulsion_tensor = 50 * torch.exp(multiplied)
    repulsion_tensor = torch.sum(0.5 * repulsion_tensor * obstacle_attraction_tensor, dim=2)
    #print(repulsion_tensor)

    # We set up tensors to determine the distance between all points in the environment and the target such that we
    # can determine the attraction force for each pixel
    attraction_tensor = torch.stack((first_layer_tensor, second_layer_tensor), dim=2)
    target_tensor = torch.tensor(target.vektor)

    # Adjust the dimesionality of target_tensor such that we can use the appropiate broadcasting functionalities 
    target_tensor.unsqueeze_(0)
    attraction_tensor = torch.sqrt(torch.sum((attraction_tensor - target_tensor) ** 2, dim=2)) / 100

    # Code for the formula given by calculate_attraction in engine_math.py: return 0.5 * target.attraction * (
    # distance_val ** 2) / (alpha * temp)
    coefficient = 0.5 * target.attraction / (alpha * temp)
    attraction_tensor = coefficient * (attraction_tensor ** 2)

    result = attraction_tensor + repulsion_tensor
    max_val = torch.max(result).item()

    # Still have to guarantee that we don't hit an obstacle it its no_interference zone. Therefore, we need to set
    # the potential field value in these anges to infinity. Atm I don't have a more efficient idea
    for obstacle in obstacles:
        x_min = max(obstacle.vektor[0] - obstacle.no_interference, 0)
        x_max = min(width, obstacle.vektor[0] + obstacle.no_interference)
        for x in range(x_min, x_max + 1):
            y_min = max(obstacle.vektor[1] - obstacle.no_interference, 0)
            y_max = min(height, obstacle.vektor[1] + obstacle.no_interference)
            for y in range(y_min, y_max + 1):
                diff = (torch.tensor(obstacle.vektor) - torch.tensor((x, y))).to(dtype=torch.float)
                if torch.linalg.norm(diff).item() <= obstacle.no_interference:
                    result[y, x] = torch.finfo(torch.float32).max

    return result


def calculate_polygon_repulsive_field_value_tensor(obstacles, width, height, alpha=1, temp=1):
    basic_tensor_row = torch.arange(0, width + 1, dtype=torch.float32)
    basic_tensor_column = torch.arange(0, height + 1, dtype=torch.float32)

    x_layer_tensor = basic_tensor_row.unsqueeze(0).repeat(height + 1, 1)
    y_layer_tensor = basic_tensor_column.unsqueeze(0).repeat(width + 1, 1).t()
    len_entries = 0
    for obstacle in obstacles:
        len_entries += len(obstacle.vertices) - 1

    a_tensor = torch.empty(len_entries * 2)
    s_tensor = torch.empty(len_entries * 2)
    n_tensor = torch.empty(len_entries * 2)

    entries = 0
    chunk_slices = ()
    max_edges_for_obstacle = 0
    for _, obstacle in enumerate(obstacles):
        for vertex_index in range(len(obstacle.vertices) - 1):
            vertex = obstacle.vertices[vertex_index]
            next_vertex = obstacle.vertices[vertex_index + 1]
            a_tensor[entries + 2 * vertex_index: entries + 2 * vertex_index + 2] = torch.tensor(vertex)
            s_tensor[entries + 2 * vertex_index: entries + 2 * vertex_index + 2] = torch.tensor(
                next_vertex) - torch.tensor(vertex)
            n_tensor[entries + 2 * vertex_index: entries + 2 * vertex_index + 2] = torch.tensor((-s_tensor[
                                                                                                  entries + 2 * vertex_index + 1: entries + 2 * vertex_index + 2],
                                                                                                 s_tensor[
                                                                                                 entries + 2 * vertex_index: entries + 2 * vertex_index + 1]))

        amount_edges = (len(obstacle.vertices) - 1)
        if max_edges_for_obstacle < amount_edges:
            max_edges_for_obstacle = amount_edges

        entries += 2 * amount_edges
        chunk_slices += (amount_edges,)

    chunk_slices_sum = ()
    cumulative_sum = 0

    for i, value in enumerate(chunk_slices):
        cumulative_sum += value
        chunk_slices_sum += (cumulative_sum,)

    n_x_tensor = n_tensor[::2]
    n_y_tensor = n_tensor[1::2]
    a_x_tensor = a_tensor[::2]
    a_y_tensor = a_tensor[1::2]
    s_x_tensor = s_tensor[::2]
    s_y_tensor = s_tensor[1::2]

    x_layer_tensor = x_layer_tensor.unsqueeze(2).expand(x_layer_tensor.size(0), x_layer_tensor.size(1), len_entries)
    y_layer_tensor = y_layer_tensor.unsqueeze(2).expand(y_layer_tensor.size(0), y_layer_tensor.size(1), len_entries)

    i_s_numerator = n_y_tensor * (a_x_tensor - x_layer_tensor) - n_x_tensor * a_y_tensor + y_layer_tensor * n_x_tensor
    i_s_denominator = s_y_tensor * n_x_tensor - s_x_tensor * n_y_tensor
    i_s = i_s_numerator / i_s_denominator

    i_s_chunked = i_s.tensor_split(chunk_slices_sum, dim=2)[:-1]

    to_be_intertwined = []
    for entry in chunk_slices:
        depth = max_edges_for_obstacle - entry
        to_add = torch.full((i_s_chunked[0].size(0), i_s_chunked[0].size(1), depth), -1)
        to_be_intertwined.append(to_add)

    result = torch.cat([torch.cat(tensors, dim=2) for tensors in zip(i_s_chunked, to_be_intertwined)], dim=2)
    reshaped = result.view(result.size(0), result.size(1), len(obstacles), -1)
    filtered_tensor = torch.where((reshaped >= 0) & (reshaped <= 1), reshaped, torch.tensor(2))

    collapsed, _ = torch.min(filtered_tensor, dim=3)

    # filtered_tensor = torch.where(collapsed == 2, torch.tensor(0), collapsed[])


def fill_infinite_polygon_repulsive_field_value(obstacles, tensor):
    for x in range(0, tensor.size(1) + 1):
        for y in range(0, tensor.size(0) + 1):
            for o in obstacles:
                # Bounding box check
                if x > o.x_max or x < o.x_min or y > o.y_max or y < o.y_min:
                    continue

                counter = 0
                for i in range(len(o.vertices)):
                    x_a = o.vertices[i][0]
                    x_b = o.vertices[i + 1][0] if (i + 1) != len(o.vertices) else o.vertices[0][0]
                    y_a = o.vertices[i][1]
                    y_b = o.vertices[i + 1][1] if (i + 1) != len(o.vertices) else o.vertices[0][1]

                    if (y < y_a) != (y < y_b):
                        if x_b - x_a == 0:
                            if x < x_a:
                                counter += 1
                        else:
                            comparing_with = x_a + ((y - y_a) / (y_b - y_a)) * (x_b - x_a)
                            if x < comparing_with:
                                counter += 1

                if counter % 2 == 1:
                    tensor[y, x] = torch.finfo(torch.float32).max
                    break

    return tensor


    """mins = [] 
    for i in range(0, (width + 1) * (height + 1)): 
        for j in range(len(offsets) - 1): 
            slice_start = i * len_entries + offsets[j]
            slice_end = i * len_entries + offsets[j + 1]
            slice_values = reshaped_i_s[slice_start:slice_end]
        
            # Filter for values between 0 and 1
            filtered_values = slice_values[(slice_values >= 0) & (slice_values <= 1)]
        
            # Find the minimum value in the filtered slice
            if filtered_values.size(0) == 0: 
                mins.append(torch.tensor(-1))
            else: 
                mins.append(torch.min(filtered_values)) 

            # TODO Should already compute the distance here. 
            # Append the minimum value to the list
        
         #print(f"The size of the i vektor is: {i_s.size()}")
    #print(y_layer_tensor.size())
    #print(n_y_tensor.size())
    """


def calculate_repulsive_force_singular_polygon_vektor(obstacles, point):
    edge_tensor_collection = []
    norm_tensor_collection = []
    a_point_tensor_collection = []
    p_point_tensor_collection = []
    storing_rows_of_obstacles = ()

    for index, obstacle in enumerate(obstacles):
        length_tensor = len(obstacle.vertices) - 1
        storing_rows_of_obstacles = storing_rows_of_obstacles + (length_tensor,)
        edge_tensor = torch.empty(size=(length_tensor, 2), dtype=torch.float32)
        norm_tensor = torch.empty(size=(length_tensor, 2), dtype=torch.float32)
        a_point_tensor = torch.empty(size=(length_tensor, 2), dtype=torch.float32)
        p_point_tensor = torch.flatten(torch.tensor(point)).repeat(length_tensor)

        for index in range(0, len(obstacle.vertices) - 1):
            point_a = torch.tensor(obstacle.vertices[index])
            edge_tensor[index] = torch.tensor(obstacle.vertices[index + 1]) - point_a
            norm_tensor[index] = torch.tensor((-edge_tensor[index, 1], edge_tensor[index, 0]))
            a_point_tensor[index] = point_a

        a_point_tensor = torch.flatten(a_point_tensor)
        edge_tensor = torch.flatten(edge_tensor)
        norm_tensor = torch.flatten(norm_tensor)

        edge_tensor_collection.append(edge_tensor)
        a_point_tensor_collection.append(a_point_tensor)
        p_point_tensor_collection.append(p_point_tensor)
        norm_tensor_collection.append(norm_tensor)

    a_point_tensor = torch.cat(a_point_tensor_collection)
    p_point_tensor = torch.cat(p_point_tensor_collection)
    #print(p_point_tensor)
    edge_tensor = torch.cat(edge_tensor_collection)
    norm_tensor = torch.cat(norm_tensor_collection)

    b = torch.t(p_point_tensor - a_point_tensor)
    edge_tensor_reshaped = torch.stack((edge_tensor, -norm_tensor), dim=1).reshape(-1, 2, 2)
    block_matrices = [torch.tensor(block) for block in edge_tensor_reshaped]

    A = torch.block_diag(*block_matrices)
    #print(A)

    solution_b = torch.linalg.solve(A, b)
    i_s = solution_b[::2]

    accumulated = 0
    solution_value = ()
    #print(solution_b)

    for i in range(len(obstacles)):
        offset = storing_rows_of_obstacles[i]
        sub_solution = i_s[accumulated:accumulated + offset]
        filtered = sub_solution[(sub_solution >= 0) & (sub_solution <= 1)]

        x = obstacles[i].vertices
        obstacle_tensor = torch.tensor(obstacles[i].vertices)
        p_point_tensor = torch.tensor(point).repeat(len(obstacles[i].vertices)).reshape(-1, 2)

        if filtered.size(0) == 0:
            #print(obstacle_tensor)
            #print(p_point_tensor[:offset+ 1].size())
            #print(p_point_tensor[:offset+ 1])
            solution_value += (
                torch.min(torch.sqrt(torch.sum((obstacle_tensor - p_point_tensor[:offset + 1]) ** 2, dim=1))).item(),)
            continue

        min_index = torch.argmin(filtered)
        z = solution_b[2 * offset + min_index * 2 + 1]
        x = norm_tensor[2 * offset + 2 * min_index:2 * offset + 2 * min_index + 2]
        distance = torch.linalg.norm(solution_b[2 * offset + min_index * 2 + 1].item() * norm_tensor[
                                                                                         2 * offset + 2 * min_index:2 * offset + 2 * min_index + 2])
        accumulated += offset
        solution_value += (distance.item(),)

    return solution_value


def main_2():
    obstacle_1 = Static_Polygon([(10, 10), (10, 610), (10, 610)], 5, 3, no_interference=2)
    obstacle_2 = Static_Polygon([(130, 10), (130, 610), (130, 610)], 5, 3, no_interference=2)
    obstacle_3 = Static_Polygon([(250, 10), (250, 610), (250, 610)], 5, 3, no_interference=2)
    obstacle_4 = Static_Polygon([(10, 10), (250, 10), (250, 10)], 5, 3, no_interference=2)
    obstacle_5 = Static_Polygon([(260, 10), (260, 610), (260, 610)], 5, 3, no_interference=2)
    obstacle_6 = Static_Polygon([(400, 10), (400, 610), (400, 610)], 5, 3, no_interference=2)
    obstacle_7 = Static_Polygon([(260, 300), (400, 300), (400, 300)], 5, 3, no_interference=2)
    obstacle_8 = Static_Polygon([(260, 10), (400, 10), (400, 10)], 5, 3, no_interference=2)
    obstacle_9 = Static_Polygon([(410, 10), (600, 10), (600, 10)], 5, 3, no_interference=2)
    obstacle_10 = Static_Polygon([(410, 10), (410, 610), (410, 610)], 5, 3, no_interference=2)
    obstacle_11 = Static_Polygon([(410, 300), (600, 300), (600, 300)], 5, 3, no_interference=2)
    obstacle_12 = Static_Polygon([(600, 10), (600, 300), (600, 300)], 5, 3, no_interference=2)
    obstacle_13 = Static_Polygon([(610, 10), (790, 10), (790, 10)], 5, 3, no_interference=2)
    obstacle_14 = Static_Polygon([(610, 10), (610, 300), (610, 300)], 5, 3, no_interference=2)
    obstacle_15 = Static_Polygon([(610, 300), (790, 300), (790, 300)], 5, 3, no_interference=2)
    obstacle_16 = Static_Polygon([(790, 300), (790, 630), (790, 630)], 5, 3, no_interference=2)
    obstacle_17 = Static_Polygon([(790, 630), (10, 630), (10, 630)], 5, 3, no_interference=2)

    obstacles = [
        obstacle_1,
        obstacle_2,
        obstacle_3,
        obstacle_4,
        obstacle_5,
        obstacle_6,
        obstacle_7,
        obstacle_8,
        obstacle_9,
        obstacle_10,
        obstacle_11,
        obstacle_12,
        obstacle_13,
        obstacle_14,
        obstacle_15,
        obstacle_16,
        obstacle_17
    ]

    s_time = time.time()

    res = calculate_polygon_repulsive_field_value_tensor(obstacles, 800, 640)
    e_time = time.time()

    print(f"It took {e_time - s_time} seconds")
    print(res)

