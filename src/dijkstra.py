import heapq
import torch
import numpy as np
from src.construct_graph import construct_graph_from_tensor


def dijkstra(start, end):
    """Summary dijkstra(): Will return the shortest path with regards to the previously computed potential field value. 

    Args:
        start (node): Starting node 
        end (node): Goal node 

    Returns: (list, cost): Will the return the path in a list of tuples corresponding to the x andd y value. The
    total cost of the path will also be returned
    """

    # Dictionary that maps node to it's current cost and parent node that expanded that node 
    distances = {start: (0, None)}

    # Set to store all already visited nodes
    visited = {start}

    priority_queue = [(0, start)]

    while priority_queue:
        # Get the node with the current least amount of costs 
        current_distance, current_node = heapq.heappop(priority_queue)

        # Backwards traversal such that we find the path if we reached the end node
        if current_node == end:
            shortest_path = []
            while current_node is not None:
                shortest_path.append(current_node.position)
                _, parent = distances[current_node]
                current_node = parent

            shortest_path.reverse()
            return shortest_path, current_distance

        for neighbor, weight in current_node.neighbors.items():
            distance = current_distance + weight

            # If we have not visited the neighbor, we need to visit him again. If we have already visited the node
            # and find a shorter path to him, we need to update the path accrodingly.
            if neighbor not in visited or distance < distances[neighbor][0]:
                distances[neighbor] = distance, current_node
                visited.add(neighbor)
                heapq.heappush(priority_queue, (distance, neighbor))

                # No path to our algorithm could be found
    return None, float('inf')


def dijkstra_on_graph_with_dictionary(tensor, start, goal):
    start_node, goal_node = construct_graph_from_tensor(tensor, start, goal, tensor.size(1), tensor.size(0))
    return dijkstra(start_node, goal_node)


def dijkstra_on_tensor_with_tensor(tensor, start, goal):
    """Summary of dijkstra_on_tensor_with_tensor(): Instead of performing Dijkstras Algorithm on an actual graph,
    we will emulate a graph. In practice this means not actually creating any nodes or edges. The weight of the edges
    to each neighbour are given by the tensor which we computed previously. The neighbour of a current node are
    simple the nodes to its left, right, up and down direction. Special edge cases may apply to the edges of the
    environment.

    Args:
        tensor ("torch.Tensor): 2D Tensor of the potetial field value function. 
        start (int, int): x and y coordinate of starting position 
        goal (int, int): y and y position of goal position
    """
    size = tuple(tensor.size())
    parent_tensor = torch.full(tuple(size) + (2,), -1, dtype=torch.int32)
    cost_tensor = torch.full(size, torch.finfo(torch.float32).max)
    cost_tensor.unsqueeze_(2)
    directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

    lookup_tensor = torch.cat((cost_tensor, parent_tensor), dim=2)

    # The sequence of start[1], start[0] is correct. It should not be start[0], start[1] due to the fact that we
    # first specify the row in tensor lookups
    lookup_tensor[start[1], start[0], 0] = 0
    priority_queue = [(0, start)]

    while priority_queue:
        cost, coordinate_tuple = heapq.heappop(priority_queue)

        # We expand the goal node, therefore we are finished and simply need to construct the path via the lookup_table
        if coordinate_tuple == goal:
            shortest_path = []
            while coordinate_tuple != (-1, -1):
                # TODO We still have to abort if the tuple cost is torch.finfo(torch.float32).max
                shortest_path.append(coordinate_tuple)
                x_coord_parent = int(lookup_tensor[coordinate_tuple[1], coordinate_tuple[0], 1].item())
                y_coord_parent = int(lookup_tensor[coordinate_tuple[1], coordinate_tuple[0], 2].item())
                coordinate_tuple = x_coord_parent, y_coord_parent

            shortest_path.reverse()
            return shortest_path, cost

        for direction in directions:
            next_x, next_y = coordinate_tuple[0] + direction[0], coordinate_tuple[1] + direction[1]

            if 0 <= next_x < tensor.size(1) and 0 <= next_y < tensor.size(0):
                distance = cost + tensor[next_y, next_x].item()

                if lookup_tensor[next_y, next_x, 0] == torch.finfo(torch.float32).max or distance < lookup_tensor[
                    next_y, next_x, 0].item():
                    lookup_tensor[next_y, next_x, 0] = distance
                    lookup_tensor[next_y, next_x, 1] = coordinate_tuple[0]
                    lookup_tensor[next_y, next_x, 2] = coordinate_tuple[1]
                    heapq.heappush(priority_queue, (distance, (next_x, next_y)))

    return None, float('inf')


def dijkstra_on_tensor_with_dictionary(tensor, start, goal):
    """Summary of dijkstra_on_tensor_with_tensor(): Instead of performing Dijkstras Algorithm on an actual graph,
    we will emulate a graph. In practice this means not actually creating any nodes or edges. The weight of the edges
    to each neighbour are given by the tensor which we computed previously. The neighbour of a current node are
    simple the nodes to its left, right, up and down direction. Special edge cases may apply to the edges of the
    environment.

    Args:
        tensor ("torch.Tensor): 2D Tensor of the potetial field value function. 
        start (int, int): x and y coordinate of starting position 
        goal (int, int): y and y position of goal position
    """
    distances = {start: (0, (-1, -1))}
    directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

    # The sequence of start[1], start[0] is correct. It should not be start[0], start[1] due to the fact that we
    # first specify the row in tensor lookups
    priority_queue = [(0, start)]

    while priority_queue:
        cost, coordinate_tuple = heapq.heappop(priority_queue)

        # We expand the goal node, therefore we are finished and simply need to construct the path via the lookup_table
        if coordinate_tuple == goal:
            shortest_path = []
            while coordinate_tuple != (-1, -1):
                # TODO We still have to abort if the tuple cost is torch.finfo(torch.float32).max
                shortest_path.append(coordinate_tuple)
                _, parent = distances[coordinate_tuple]
                coordinate_tuple = parent

            shortest_path.reverse()
            return shortest_path, cost

        for direction in directions:
            next_x, next_y = coordinate_tuple[0] + direction[0], coordinate_tuple[1] + direction[1]

            if 0 <= next_x < tensor.size(1) and 0 <= next_y < tensor.size(0):
                distance = cost + tensor[next_y, next_x].item()

                if (next_x, next_y) not in distances or distance < distances[(next_x, next_y)][0]:
                    distances[(next_x, next_y)] = distance, coordinate_tuple
                    heapq.heappush(priority_queue, (distance, (next_x, next_y)))

    return None, float('inf')


def dijkstra_on_tensor_with_array(tensor, start, goal):
    lookup_array = np.full((3, tensor.size(0), tensor.size(1)), -1)
    lookup_array[0, start[1], start[0]] = 0
    directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]
    priority_queue = [(0, start)]

    while priority_queue:
        cost, coordinate_tuple = heapq.heappop(priority_queue)

        # We expand the goal node, therefore we are finished and simply need to construct the path via the lookup_table
        if coordinate_tuple == goal:
            shortest_path = []
            while coordinate_tuple != (-1, -1):
                # TODO We still have to abort if the tuple cost is torch.finfo(torch.float32).max
                shortest_path.append(coordinate_tuple)
                coordinate_tuple = (lookup_array[1, coordinate_tuple[1], coordinate_tuple[0]],
                                    lookup_array[2, coordinate_tuple[1], coordinate_tuple[0]])

            shortest_path.reverse()
            return shortest_path, cost

        for direction in directions:
            next_x, next_y = coordinate_tuple[0] + direction[0], coordinate_tuple[1] + direction[1]

            if 0 <= next_x < tensor.size(1) and 0 <= next_y < tensor.size(0):
                distance = cost + tensor[next_y, next_x].item()

                current_cost = lookup_array[0, next_y, next_x]
                if current_cost == -1 or distance < current_cost:
                    lookup_array[0, next_y, next_x] = distance
                    lookup_array[1, next_y, next_x] = coordinate_tuple[0]
                    lookup_array[2, next_y, next_x] = coordinate_tuple[1]
                    heapq.heappush(priority_queue, (distance, (next_x, next_y)))

    return None, float('inf')


def dijkstra_on_nparray_with_dictionary(tensor, start, goal):
    """Summary of dijkstra_on_tensor_with_tensor(): Instead of performing Dijkstras Algorithm on an actual graph,
    we will emulate a graph. In practice this means not actually creating any nodes or edges. The weight of the edges
    to each neighbour are given by the tensor which we computed previously. The neighbour of a current node are
    simple the nodes to its left, right, up and down direction. Special edge cases may apply to the edges of the
    environment.

    Args:
        tensor ("torch.Tensor): 2D Tensor of the potetial field value function. 
        start (int, int): x and y coordinate of starting position 
        goal (int, int): y and y position of goal position
    """
    np_array = tensor.detach().numpy()
    distances = {start: (0, (-1, -1))}
    directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

    # The sequence of start[1], start[0] is correct. It should not be start[0], start[1] due to the fact that we
    # first specify the row in tensor lookups
    priority_queue = [(0, start)]

    while priority_queue:
        cost, coordinate_tuple = heapq.heappop(priority_queue)

        # We expand the goal node, therefore we are finished and simply need to construct the path via the lookup_table
        if coordinate_tuple == goal:
            shortest_path = []
            while coordinate_tuple != (-1, -1):
                if np_array[coordinate_tuple[1], coordinate_tuple[0]] == torch.finfo(torch.float32).max:
                    exit("No path to the goal that does not collide with obstacle!")
                shortest_path.append(coordinate_tuple)
                _, parent = distances[coordinate_tuple]
                coordinate_tuple = parent

            shortest_path.reverse()
            return shortest_path, cost

        for direction in directions:
            next_x, next_y = coordinate_tuple[0] + direction[0], coordinate_tuple[1] + direction[1]

            if 0 <= next_x < tensor.size(1) and 0 <= next_y < tensor.size(0):
                distance = cost + np_array[next_y, next_x]

                if (next_x, next_y) not in distances or distance < distances[(next_x, next_y)][0]:
                    distances[(next_x, next_y)] = distance, coordinate_tuple
                    heapq.heappush(priority_queue, (distance, (next_x, next_y)))

    return None, float('inf')


def dijkstra_on_nparray_with_dictionary_without_detach(tensor, start, goal):
    """Summary of dijkstra_on_tensor_with_tensor(): Instead of performing Dijkstras Algorithm on an actual graph,
    we will emulate a graph. In practice this means not actually creating any nodes or edges. The weight of the edges
    to each neighbour are given by the tensor which we computed previously. The neighbour of a current node are
    simple the nodes to its left, right, up and down direction. Special edge cases may apply to the edges of the
    environment.

    Args:
        tensor ("torch.Tensor): 2D Tensor of the potetial field value function. 
        start (int, int): x and y coordinate of starting position 
        goal (int, int): y and y position of goal position
    """
    np_array = tensor.numpy()
    distances = {start: (0, (-1, -1))}
    directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

    # The sequence of start[1], start[0] is correct. It should not be start[0], start[1] due to the fact that we
    # first specify the row in tensor lookups
    priority_queue = [(0, start)]

    while priority_queue:
        cost, coordinate_tuple = heapq.heappop(priority_queue)

        # We expand the goal node, therefore we are finished and simply need to construct the path via the lookup_table
        if coordinate_tuple == goal:
            shortest_path = []
            while coordinate_tuple != (-1, -1):
                if np_array[coordinate_tuple[1], coordinate_tuple[0]] == torch.finfo(torch.float32).max:
                    exit("No path to the goal that does not collide with obstacle!")
                shortest_path.append(coordinate_tuple)
                _, parent = distances[coordinate_tuple]
                coordinate_tuple = parent

            shortest_path.reverse()
            return shortest_path, cost

        for direction in directions:
            next_x, next_y = coordinate_tuple[0] + direction[0], coordinate_tuple[1] + direction[1]
            if 0 <= next_x < tensor.size(1) and 0 <= next_y < tensor.size(0):
                distance = cost + np_array[next_y, next_x]

                if (next_x, next_y) not in distances or distance < distances[(next_x, next_y)][0]:
                    distances[(next_x, next_y)] = distance, coordinate_tuple
                    heapq.heappush(priority_queue, (distance, (next_x, next_y)))

    return None, float('inf')


def dijkstra_on_nparray_with_array(tensor, start, goal, ):
    np_array = tensor.detach().numpy()
    lookup_array = np.full((3, tensor.size(0), tensor.size(1)), -1)
    lookup_array[0, start[1], start[0]] = 0
    directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]
    priority_queue = [(0, start)]

    while priority_queue:
        cost, coordinate_tuple = heapq.heappop(priority_queue)

        # We expand the goal node, therefore we are finished and simply need to construct the path via the lookup_table
        if coordinate_tuple == goal:
            shortest_path = []
            while coordinate_tuple != (-1, -1):
                if (np_array[coordinate_tuple[1], coordinate_tuple[0]] == torch.finfo(torch.float32).max):
                    exit("No path to the goal that does not collide with obstacle!")
                shortest_path.append(coordinate_tuple)
                coordinate_tuple = (lookup_array[1, coordinate_tuple[1], coordinate_tuple[0]],
                                    lookup_array[2, coordinate_tuple[1], coordinate_tuple[0]])

            shortest_path.reverse()
            return shortest_path, cost

        for direction in directions:
            next_x, next_y = coordinate_tuple[0] + direction[0], coordinate_tuple[1] + direction[1]

            if 0 <= next_x < tensor.size(1) and 0 <= next_y < tensor.size(0):
                distance = cost + np_array[next_y, next_x]

                current_cost = lookup_array[0, next_y, next_x]
                if current_cost == -1 or distance < current_cost:
                    lookup_array[0, next_y, next_x] = distance
                    lookup_array[1, next_y, next_x] = coordinate_tuple[0]
                    lookup_array[2, next_y, next_x] = coordinate_tuple[1]
                    heapq.heappush(priority_queue, (distance, (next_x, next_y)))

    return None, float('inf')


def dijkstra_on_nparray_with_two_array(tensor, start, goal):
    np_array = tensor.detach().numpy()
    cost_array = np.full((tensor.size(0), tensor.size(1)), -1)
    parent_array = np.full((2, tensor.size(0), tensor.size(1)), -1, dtype=np.int32)
    cost_array[start[1], start[0]] = 0
    directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]
    priority_queue = [(0, start)]

    while priority_queue:
        cost, coordinate_tuple = heapq.heappop(priority_queue)

        # We expand the goal node, therefore we are finished and simply need to construct the path via the lookup_table
        if coordinate_tuple == goal:
            shortest_path = []
            while coordinate_tuple != (-1, -1):
                if np_array[coordinate_tuple[1], coordinate_tuple[0]] == torch.finfo(torch.float32).max:
                    exit("No path to the goal that does not collide with obstacle!")

                shortest_path.append(coordinate_tuple)
                coordinate_tuple = (parent_array[0, coordinate_tuple[1], coordinate_tuple[0]],
                                    parent_array[1, coordinate_tuple[1], coordinate_tuple[0]])

            shortest_path.reverse()
            return shortest_path, cost

        for direction in directions:
            next_x, next_y = coordinate_tuple[0] + direction[0], coordinate_tuple[1] + direction[1]

            if 0 <= next_x < tensor.size(1) and 0 <= next_y < tensor.size(0):
                distance = cost + np_array[next_y, next_x]

                current_cost = cost_array[next_y, next_x]
                if current_cost == -1 or distance < current_cost:
                    cost_array[next_y, next_x] = distance
                    parent_array[0, next_y, next_x] = coordinate_tuple[0]
                    parent_array[1, next_y, next_x] = coordinate_tuple[1]
                    heapq.heappush(priority_queue, (distance, (next_x, next_y)))

    return None, float('inf')
