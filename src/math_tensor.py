import torch
from classes.staticpolygon import StaticPolygon
from classes.staticcircle import StaticCircle


C_DOUBLE_PRIME = -0.001
C = 100


def calculate_total_repulsive_field_value(obstacles, target, width, height, alpha=1, temp=1):
    """
    Summary of create_total_repulsive_field_value(): Will calculate the total repulsive field value for
    the given obstacles. Will call individual repulsive field value functions for polygons and circles as well as the
    attraction force for the target.

    Args:
        obstacles (list): list of obstacles in given environment.
        target (StaticCircle): target in given environment.
        width (int): width of the environment.
        height (int): height of the environment.
        alpha (float): alpha for deterministic annealing. Defaults to 1.
        temp (float): temp for deterministic annealing. Defaults to 1.

    Returns: Tensor with total potential field value.
    """

    circle_obstacle_list = []
    polygon_obstacle_list = []

    # Separation into obstacles of type Circle and Polygon
    for obstacle in obstacles:
        if isinstance(obstacle, StaticPolygon):
            polygon_obstacle_list.append(obstacle)
        elif isinstance(obstacle, StaticCircle):
            circle_obstacle_list.append(obstacle)

    polygon_distance_tensor = None
    circle_distance_tensor = None
    if len(circle_obstacle_list) != 0:
        circle_distance_tensor = calculate_circle_distance_tensor(circle_obstacle_list, width, height)
    if len(polygon_obstacle_list) != 0:
        polygon_distance_tensor = calculate_polygon_distance_tensor(polygon_obstacle_list, width, height)

    attraction_repulsive_tensor = calculate_attraction_field_value_tensor(target, width, height, alpha, temp)

    if len(circle_obstacle_list) != 0:
        attraction_repulsive_tensor += calculate_repulsive_field_value(circle_distance_tensor, circle_obstacle_list)

    if len(polygon_obstacle_list) != 0:
        attraction_repulsive_tensor += calculate_repulsive_field_value(polygon_distance_tensor, polygon_obstacle_list)

    intermed = fill_infinite_circle_repulsive_field_value(circle_obstacle_list, attraction_repulsive_tensor)
    result = fill_infinite_polygon_repulsive_field_value(polygon_obstacle_list, intermed, polygon_distance_tensor)

    return result


def calculate_attraction_field_value_tensor(target, width, height, alpha=1, temp=1):
    """
    Summary of calculate_attraction_field_value_tensor(): Will calculate the total attraction field value
    tensor.

    Args:
        target (StaticCircle): target in given environment.
        width (int): width of the environment.
        height (int): height of the environment.
        alpha (float): alpha for deterministic annealing. Defaults to 1.
        temp (float): temp for deterministic annealing. Defaults to 1.

    Returns: A tensor with the attraction field value.
    """

    x_layer_tensor, y_layer_tensor = create_base_tensors(width, height)

    # We set up tensors to determine the distance between all points in the environment and the target such that we
    # can determine the attraction force for each pixel
    attraction_tensor = torch.stack((x_layer_tensor, y_layer_tensor), dim=2)
    target_tensor = torch.tensor(target.vector)

    # Adjust the dimensionality of target_tensor such that we can use the appropriate broadcasting functionalities
    target_tensor.unsqueeze_(0)
    attraction_tensor = torch.sqrt(torch.sum((attraction_tensor - target_tensor) ** 2, dim=2)) / 100

    # Code for the formula given by calculate_attraction in engine_math.py: return 0.5 * target.attraction * (
    # distance_val ** 2) / (alpha * temp)
    coefficient = 0.5 * target.attraction / (alpha * temp)
    attraction_tensor = coefficient * (attraction_tensor ** 2)
    return attraction_tensor


def calculate_repulsive_field_value(distance_tensor, obstacles, alpha=1, temp=1):
    """
    Summary of calculate_repulsive_field_value(): Will calculate the total repulsive field value given the distance
     tensor.

    Args:
        distance_tensor (torch.Tensor): distance tensor for either all circles or all polygons.
        obstacles (list): partial list obstacles in given environment. This partial list contains either all Circles or
        all Polygons obstacles.
        alpha (float): alpha for deterministic annealing. Defaults to 1.
        temp (float): temp for deterministic annealing. Defaults to 1.

    Returns:
        2D tensor with total repulsive field value for either all circles or all polygons. Depends on the obstacles
        given to the function calculate_total_repulsive_field_value().
    """

    # 1D tensor which will only store the attraction values of each individual obstacle. The order of the obstacles
    # in  obstacle_position_tensor and obstacle_attraction_tensor must be the same.
    obstacle_distance_of_influence_tensor = torch.empty(len(obstacles))
    obstacle_attraction_tensor = torch.empty(len(obstacles))

    for index, obstacle in enumerate(obstacles):
        obstacle_distance_of_influence_tensor[index] = torch.tensor(obstacle.distance_of_influence).to(
            obstacle_distance_of_influence_tensor.dtype)
        obstacle_attraction_tensor = torch.tensor(obstacle.attraction).to(obstacle_distance_of_influence_tensor.dtype)
    # General formula for repulsive formula: c' * math.exp(c'' * c/obstacle.distance_of_influence * (
    # distance_by_alptemp ** 2)) Here we calculate everything except for the distance_by_alptemp ** 2 in the exponent
    obstacle_distance_of_influence_tensor = (C_DOUBLE_PRIME * C) / obstacle_distance_of_influence_tensor

    obstacle_distance_of_influence_tensor.unsqueeze_(0).unsqueeze_(0)

    alpha_temp = distance_tensor / (alpha * temp)
    squared_again = alpha_temp ** 2

    multiplied = squared_again * obstacle_distance_of_influence_tensor
    repulsion_tensor = 50 * torch.exp(multiplied)
    repulsion_tensor = torch.sum(0.5 * repulsion_tensor * obstacle_attraction_tensor, dim=2)

    return repulsion_tensor


def calculate_circle_distance_tensor(obstacles, width, height):
    """
    Summary of calculate_circle_distance_tensor(): This function will calculate the distance to each.
    circle obstacle for all points in parallel using PyTorch.
       
    Args:
        obstacles (list): list of obstacles in given environment.
        width (int): width of the environment.
        height (int): height of the environment.

    Returns:
        3D-tensor: Will return a 3D tensor of the distances of each circle obstacle to each pixel.
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
    obstacle_no_interference_offset = torch.empty(len(obstacles))

    for index, obstacle in enumerate(obstacles):
        obstacle_position_tensor[index] = torch.tensor(obstacle.vector).to(obstacle_position_tensor.dtype)
        obstacle_distance_of_influence_tensor[index] = torch.tensor(obstacle.distance_of_influence).to(
            obstacle_distance_of_influence_tensor.dtype)
        obstacle_attraction_tensor = torch.tensor(obstacle.attraction).to(obstacle_distance_of_influence_tensor.dtype)
        obstacle_no_interference_offset[index] = torch.tensor(obstacle.no_interference)

    # General formula for repulsive formula: c' * math.exp(c'' * c/obstacle.distance_of_influence * (
    # distance_by_alptemp ** 2)) Here we calculate everything except for the distance_by_alptemp ** 2 in the exponent
    obstacle_distance_of_influence_tensor = (C_DOUBLE_PRIME * C) / obstacle_distance_of_influence_tensor

    position_tensor.unsqueeze_(2)
    obstacle_position_tensor.unsqueeze_(0).unsqueeze_(0)
    obstacle_distance_of_influence_tensor.unsqueeze_(0).unsqueeze_(0)
    obstacle_attraction_tensor.unsqueeze_(0).unsqueeze_(0)

    # After adjusting the dimensions we are now able to perform operations to determine the Euclidean distance to all
    # obstacle. The distance of each position (x,y) to a respective object j will be stored in the 4th dimension
    difference = position_tensor - obstacle_position_tensor
    squared = difference ** 2

    # Collapsing the tensor in the 4th dimension. For every object j only the Euclidean distance from point (x,
    # y) to it will be stored in the cell (x,y,j)
    sum = torch.sum(squared, dim=3)
    result = torch.sqrt(sum)

    result = result - obstacle_no_interference_offset

    return result


def calculate_polygon_distance_tensor(obstacles, width, height):
    """
    Summary of  calculate_polygon_distance_tensor(): This function will calculate the distance to each
    polygon obstacle for all points in parallel using PyTorch.

    Look at either the documentation or the slides of the final presentation to understand the math concept. These can
    be found in the folder `report`.

    Args:
        obstacles (list): list of all polygons in given environment
        width (int): width of the environment.
        height (int): height of the environment.

    Returns:
        3D-tensor: Will return a 3D tensor of the distances of each circle obstacle to each pixel.
    """

    x_layer_tensor, y_layer_tensor = create_base_tensors(width, height)
    stacked_tensor = torch.stack((x_layer_tensor, y_layer_tensor, x_layer_tensor, y_layer_tensor), dim=2)

    # Calculate the total amount of vertices in our environment
    len_entries = 0
    for obstacle in obstacles:
        len_entries += len(obstacle.vertices)

    a_tensor = torch.empty(len_entries * 2)
    # s_tensor can also be referred to the ab_tensor
    s_tensor = torch.empty(len_entries * 2)
    n_tensor = torch.empty(len_entries * 2)
    coordinates_tensor = torch.empty(len_entries, 4)

    # Storing the amount of vertices for each polygon in chunk_slices. This will be used to chunk the tensor later on
    # and adding fictive dimensionality
    entries = 0
    chunk_slices = ()
    max_edges_for_obstacle = 0
    for _, obstacle in enumerate(obstacles):
        for vertex_index in range(len(obstacle.vertices)):
            vertex = obstacle.vertices[vertex_index]
            # We need a case distinction for the wrap around
            next_vertex = obstacle.vertices[vertex_index + 1] if vertex_index < (len(obstacle.vertices) - 1) \
                else obstacle.vertices[0]
            vertex = torch.tensor(vertex)
            next_vertex = torch.tensor(next_vertex)
            concatenated_tensor = torch.cat((vertex, next_vertex))
            coordinates_tensor[entries // 2 + vertex_index] = concatenated_tensor
            a_tensor[entries + 2 * vertex_index: entries + 2 * vertex_index + 2] = vertex
            s_tensor[entries + 2 * vertex_index: entries + 2 * vertex_index + 2] = next_vertex - vertex
            n_tensor[entries + 2 * vertex_index: entries + 2 * vertex_index + 2] = torch.tensor((-s_tensor[entries + 2 *
                                                                                                           vertex_index + 1: entries + 2 * vertex_index + 2],
                                                                                                 s_tensor[
                                                                                                 entries + 2 * vertex_index: entries + 2 *
                                                                                                                             vertex_index + 1]))

        amount_edges = (len(obstacle.vertices))
        if max_edges_for_obstacle < amount_edges:
            max_edges_for_obstacle = amount_edges

        entries += 2 * amount_edges
        chunk_slices += (amount_edges,)

    chunk_slices_sum = ()
    cumulative_sum = 0

    # Calculation of the slicing / chunking indices in the third dimension
    for i, value in enumerate(chunk_slices):
        cumulative_sum += value
        chunk_slices_sum += (cumulative_sum,)

    # Separation of tensor values in x and y coordinates respectively
    n_x_tensor = n_tensor[::2]
    n_y_tensor = n_tensor[1::2]
    a_x_tensor = a_tensor[::2]
    a_y_tensor = a_tensor[1::2]
    s_x_tensor = s_tensor[::2]
    s_y_tensor = s_tensor[1::2]

    n_distance_tensor = torch.sqrt((n_x_tensor ** 2) + (n_y_tensor ** 2))
    n_distance_tensor = n_distance_tensor.unsqueeze_(0).unsqueeze_(0).expand(height + 1, width + 1, cumulative_sum)

    # Need to perform operations on the tensor such that broadcasting will work
    x_layer_tensor = x_layer_tensor.unsqueeze(2).expand(x_layer_tensor.size(0), x_layer_tensor.size(1), len_entries)
    y_layer_tensor = y_layer_tensor.unsqueeze(2).expand(y_layer_tensor.size(0), y_layer_tensor.size(1), len_entries)

    # Look at documentation / slides for derivation of following two equations
    i_s_numerator = n_y_tensor * (a_x_tensor - x_layer_tensor) - n_x_tensor * a_y_tensor + y_layer_tensor * n_x_tensor
    i_s_denominator = s_y_tensor * n_x_tensor - s_x_tensor * n_y_tensor
    i_s = i_s_numerator / i_s_denominator

    j_s_numerator = (a_y_tensor * s_x_tensor + x_layer_tensor * s_y_tensor - y_layer_tensor * s_x_tensor - a_x_tensor *
                     s_y_tensor)
    j_s_denominator = n_y_tensor * s_x_tensor - n_x_tensor * s_y_tensor
    j_s = torch.abs(j_s_numerator / j_s_denominator)

    distance_to_edge = j_s * n_distance_tensor

    stacked_tensor = stacked_tensor.unsqueeze_(2).expand(stacked_tensor.size(0), stacked_tensor.size(1), cumulative_sum,
                                                         4)
    # Calculation of tensor that stores the distances to the closest vertex of an edge. This tensor will also have
    # fictive entries as well
    distance_to_minimum_vertex = (stacked_tensor - coordinates_tensor) ** 2
    distance_to_minimum_vertex = distance_to_minimum_vertex.view(distance_to_minimum_vertex.size(0),
                                                                 distance_to_minimum_vertex.size(1), cumulative_sum,
                                                                 2, 2)

    distance_to_minimum_vertex = torch.sum(distance_to_minimum_vertex, dim=4)
    distance_to_minimum_vertex = torch.sqrt(distance_to_minimum_vertex)
    distance_to_minimum_vertex, _ = torch.min(distance_to_minimum_vertex, dim=3)

    # Splitting them such that we can add fictive dimensions in order to extract a fourth dimension. We disregard the
    # last splice because it's emtpy. It keeps all the tensors that did not fit in the slices before. Because we do
    # exact slicing, this last "bucket" is not needed.

    i_s_chunked = i_s.tensor_split(chunk_slices_sum, dim=2)[:-1]
    distances_to_edge_chunked = distance_to_edge.tensor_split(chunk_slices_sum, dim=2)[:-1]
    distances_to_vertex_chunked = distance_to_minimum_vertex.tensor_split(chunk_slices_sum, dim=2)[:-1]

    i_s_to_be_intertwined = []
    distances_edge_to_be_intertwined = []
    distances_vertex_to_be_intertwined = []
    for entry in chunk_slices:
        depth = max_edges_for_obstacle - entry

        # The fictive dimensions should not be considered while evaluating the distance to each obstacle. Therefore,
        # when we take the minimum over all distances to each vertex, these values should be disregarded. This is
        # achieved by setting the distance to the fictive entry to zero. The distance of infinity is stored in the
        # distance_to_vertex_add tensor. So that masking works correctly, the filer value, i, must be set to a value in
        # the range [0, 1]. We simply set it to 1.

        i_to_add = torch.full((i_s_chunked[0].size(0), i_s_chunked[0].size(1), depth), 1)
        distances_to_edge_add = torch.full((distances_to_edge_chunked[0].size(0), distances_to_edge_chunked[0].size(1), depth),
                                      torch.finfo(torch.float32).max)
        distances_to_vertex_add = torch.full((distances_to_vertex_chunked[0].size(0), distances_to_vertex_chunked[0].size(1), depth),
                                             torch.finfo(torch.float32).max)

        i_s_to_be_intertwined.append(i_to_add)
        distances_edge_to_be_intertwined.append(distances_to_edge_add)
        distances_vertex_to_be_intertwined.append(distances_to_vertex_add)

    i_s_result = torch.cat([torch.cat(tensors, dim=2) for tensors in zip(i_s_chunked, i_s_to_be_intertwined)], dim=2)
    i_s_reshaped = i_s_result.view(i_s_result.size(0), i_s_result.size(1), len(obstacles), -1)

    distances_to_edge_result = torch.cat(
        [torch.cat(tensors, dim=2) for tensors in zip(distances_to_edge_chunked, distances_edge_to_be_intertwined)], dim=2)
    distances_edge_reshaped = distances_to_edge_result.view(distances_to_edge_result.size(0), distances_to_edge_result.size(1), len(obstacles), -1)

    distances_to_vertex_result = torch.cat(
        [torch.cat(tensors, dim=2) for tensors in zip(distances_to_vertex_chunked, distances_vertex_to_be_intertwined)], dim=2
    )
    distances_vertex_reshaped = distances_to_vertex_result.view(distances_to_vertex_result.size(0), distances_to_vertex_result.size(1), len(obstacles), -1)

    # We filter depending on the i-value
    filtered_tensor = torch.where((i_s_reshaped >= 0) & (i_s_reshaped <= 1), distances_edge_reshaped, distances_vertex_reshaped)

    min_distance_tensor, _ = torch.min(filtered_tensor, dim=3)
    return min_distance_tensor


def fill_infinite_circle_repulsive_field_value(obstacles, tensor):
    """
    Summary of fill_infinite_circle_repulsive_field_value(): Still have to guarantee that we don't hit a circle
    obstacle it its no_interference zone. Therefore, we need to set the potential field value in these coordinate to
    infinity.

    Args:
        obstacles (list): list of all circle obstacles in the environment.
        tensor (torch.Tensor): 2D repulsive field tensor without infinite values.

    Returns: 2D repulsive field value tensor with infinite values for collision avoidance.
    """

    if len(obstacles) == 0:
        return tensor

    for obstacle in obstacles:
        x_min = max(obstacle.vector[0] - obstacle.no_interference, 0)
        x_max = min(tensor.size(1), obstacle.vector[0] + obstacle.no_interference)
        for x in range(x_min, x_max + 1):
            y_min = max(obstacle.vector[1] - obstacle.no_interference, 0)
            y_max = min(tensor.size(0), obstacle.vector[1] + obstacle.no_interference)
            for y in range(y_min, y_max + 1):
                diff = (torch.tensor(obstacle.vector) - torch.tensor((x, y))).to(dtype=torch.float)

                # Use Euclidean norm to determine whether we need to set the repulsive field value to infinity
                if torch.linalg.norm(diff).item() <= obstacle.no_interference:
                    tensor[y, x] = torch.finfo(torch.float32).max

    return tensor


def fill_infinite_polygon_repulsive_field_value(obstacles, tensor, min_distance_tensor):
    """
      Summary of fill_infinite_polygon_repulsive_field_value(): Still have to guarantee that we don't hit a
      polygon obstacle it its no_interference zone. Therefore, we need to set the potential field value in these
      coordinate to infinity.

      Args:
          min_distance_tensor (torch.Tensor): 3D tensor which stores the minimum distance of all points to all polygon.
          obstacles.
          obstacles (list): list of all polygon obstacles in the environment.
          tensor (torch.Tensor): 2D repulsive field tensor without infinite values.

      Returns: 2D repulsive field value tensor with infinite values for collision avoidance.
      """

    if len(obstacles) == 0:
        return tensor

    np_min_dist = min_distance_tensor.detach().numpy()

    for x in range(0, tensor.size(1)):
        for y in range(0, tensor.size(0)):
            for index, o in enumerate(obstacles):
                # Bounding box check
                if x > o.x_max + o.no_interference or x < o.x_min - o.no_interference or y > o.y_max + o.no_interference or y < o.y_min - o.no_interference:
                    continue

                if np_min_dist[y, x, index] <= o.no_interference:
                    tensor[y, x] = torch.finfo(torch.float32).max
                    break

                counter = 0

                # Use ray casting algorithm  to determine whether a points reside inside a polygon, and we have to set
                # to infinity

                for i in range(len(o.vertices)):
                    x_a = o.vertices[i][0]
                    x_b = o.vertices[i + 1][0] if (i + 1) != len(o.vertices) else o.vertices[0][0]
                    y_a = o.vertices[i][1]
                    y_b = o.vertices[i + 1][1] if (i + 1) != len(o.vertices) else o.vertices[0][1]

                    """Got the idea of choosing the ray casting point and the conditions that must be checked from here:
                    https://www.youtube.com/watch?v=RSXM9bgqxJM&t=271s 4:32"""

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
    """
    Summary of create base_tensors(): Create a base tensor with the x and y sheet respective the given width and height.

    Args:
        width (int): width of the environment.
        height (int): height of the environment.

    Returns:
        3D tensor of size height x width x 2.
    """

    basic_tensor_row = torch.arange(0, width + 1, dtype=torch.float32)
    basic_tensor_column = torch.arange(0, height + 1, dtype=torch.float32)

    x_layer_tensor = basic_tensor_row.unsqueeze(0).repeat(height + 1, 1)
    y_layer_tensor = basic_tensor_column.unsqueeze(0).repeat(width + 1, 1).t()

    return x_layer_tensor, y_layer_tensor
