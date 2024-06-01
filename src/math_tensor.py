import torch

C_DOUBLE_PRIME = -0.001
C = 100


def calculate_total_repulsive_field_value(obstacles, target, width, height, alpha=1, temp=1):

    circle_repulsive_tensor = calculate_circle_repulsive_field_value_tensor(obstacles, width, height, alpha, temp)
    attraction_repulsive_tensor = calculate_attraction_field_value_tensor(target, width, height, alpha, temp)

    non_infinity_result = circle_repulsive_tensor + attraction_repulsive_tensor

    return fill_infinite_circle_repulsive_field_value(obstacles, non_infinity_result)


def calculate_attraction_field_value_tensor(target, width, height, alpha=1, temp=1):
    """
    Calculates attraction field value tensor given target and width and height.
    """
    x_layer_tensor, y_layer_tensor = create_base_tensors(width, height)
    # We set up tensors to determine the distance between all points in the environment and the target such that we
    # can determine the attraction force for each pixel
    attraction_tensor = torch.stack((x_layer_tensor, y_layer_tensor), dim=2)
    target_tensor = torch.tensor(target.vector)

    # Adjust the dimesionality of target_tensor such that we can use the appropiate broadcasting functionalities
    target_tensor.unsqueeze_(0)
    attraction_tensor = torch.sqrt(torch.sum((attraction_tensor - target_tensor) ** 2, dim=2)) / 100

    # Code for the formula given by calculate_attraction in engine_math.py: return 0.5 * target.attraction * (
    # distance_val ** 2) / (alpha * temp)
    coefficient = 0.5 * target.attraction / (alpha * temp)
    attraction_tensor = coefficient * (attraction_tensor ** 2)
    return attraction_tensor


def calculate_circle_repulsive_field_value_tensor(obstacles, width, height, alpha=1, temp=1):
    """Summary of function create_tensor_repulsive_force(): This function will calculate the potential field value
    for the entire obstacle and store the value of each pixel (x,y) in a tensor. Calculating the potential field value
    function will be done through Pytorch. This will automatically result in parallel execution
       
    Args:
        obstacles (list): list of obstacles in given environment
        width (int): width of the environment
        height (int): height of the environment 
        alpha (float): alpha for deterministic annealing. Defaults to 1.
        temp (float): temp for deterministic annealing. Defaults to 1.


    Returns:
        2D-tensor: Will return a 2D tensor of the potential field value 
    """

    x_layer_tensor, y_layer_tensor = create_base_tensors(width, height)

    # 3D tensor with the dimensions (width, height,2) with x and y coordinates
    position_tensor = torch.stack((x_layer_tensor, y_layer_tensor), dim=2)

    # 2D tensor with the x and y coordinate of each obstacle in it 
    obstacle_position_tensor = torch.empty(len(obstacles), 2)

    # 1D tensor which will only store the attraction values of each individual obstacle. The order of the obstacles
    # in  obstacle_position_tensor and obstacle_attraction_tensor must be the same.
    obstacle_distance_of_influence_tensor = torch.empty(len(obstacles))
    obstacle_attraction_tensor = torch.empty(len(obstacles))

    for index, obstacle in enumerate(obstacles):
        obstacle_position_tensor[index] = torch.tensor(obstacle.vector).to(obstacle_position_tensor.dtype)
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
    difference = position_tensor - obstacle_position_tensor
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

    return repulsion_tensor


def calculate_polygon_repulsive_field_value_tensor(obstacles, width, height, alpha=1, temp=1):
    x_layer_tensor, y_layer_tensor = create_base_tensors(width, height)

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
            n_tensor[entries + 2 * vertex_index: entries + 2 * vertex_index + 2] = torch.tensor((-s_tensor[entries + 2 *
                vertex_index + 1: entries + 2 * vertex_index + 2],s_tensor[entries + 2 * vertex_index: entries + 2 *
                vertex_index + 1]))

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


def fill_infinite_circle_repulsive_field_value(obstacles, tensor):
    # Still have to guarantee that we don't hit an obstacle it its no_interference zone. Therefore, we need to set
    # the potential field value in these anges to infinity. Atm I don't have a more efficient idea
    for obstacle in obstacles:
        x_min = max(obstacle.vector[0] - obstacle.no_interference, 0)
        x_max = min(tensor.size(1), obstacle.vector[0] + obstacle.no_interference)
        for x in range(x_min, x_max + 1):
            y_min = max(obstacle.vector[1] - obstacle.no_interference, 0)
            y_max = min(tensor.size(0), obstacle.vector[1] + obstacle.no_interference)
            for y in range(y_min, y_max + 1):
                diff = (torch.tensor(obstacle.vector) - torch.tensor((x, y))).to(dtype=torch.float)
                if torch.linalg.norm(diff).item() <= obstacle.no_interference:
                    tensor[y, x] = torch.finfo(torch.float32).max

    return tensor


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


def create_base_tensors(width, height):
    basic_tensor_row = torch.arange(0, width + 1, dtype=torch.float32)
    basic_tensor_column = torch.arange(0, height + 1, dtype=torch.float32)

    x_layer_tensor = basic_tensor_row.unsqueeze(0).repeat(height + 1, 1)
    y_layer_tensor = basic_tensor_column.unsqueeze(0).repeat(width + 1, 1).t()

    return x_layer_tensor, y_layer_tensor
