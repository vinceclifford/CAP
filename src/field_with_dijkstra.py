from src.node import Node
from src.robot import Robot
from src.staticcircle import StaticCircle
from src.math_engine import calculate_potential_field_value_temperature
from src.dijkstra import dijkstra
from tqdm import tqdm


def pathplanning_with_potential_field_and_dijkstra(robot, target, obstacles, width, height, alpha=1, temp=1):
    """Summary of function pathplanning_with_potential_field_and_dijkstra: Will use dijkstra's algorithm to solve the
    pathplanning problem.'

    Args:
        robot (static_circle): Object of the robot.
        target (static_circle): Object of the goal.
        obstacles (list): list of obstacles. An obstacle could either be of type static_circle or static_polygon.
        width (int): screen width. 
        height (int): screen height. 
        alpha (float): alpha for deterministic annealing. Default is 1.
        temp (float): temp for deterministic annealing. Default is 1.

    Returns: ((list, float)): Returns a tuple. The first entry in the tuple is a list of tuples. Each tuple contains
    two integer coordinates which represent a point the robot will be at. All points together will represent the path
    the robot will take. The second entry of the tuple the function returns is the sum of all potential field values
    along the taken path.
    """
    start, goal = construct_entire_graph(robot, target, obstacles, width, height, int(alpha), int(temp))
    path, cost = dijkstra(start, goal)
    return path, cost


def construct_entire_graph(robot, target: StaticCircle, obstacles, width, height, alpha=1, temp=1):
    """Summary of construct_entire_graph(): Creates an explicit graph representation of the environment.

    Args:
        Same arguments as in function pathplanning_with_potential_field_and_dijkstra.

    Returns:
        (node, node): Returns the starting and goal node of the graph. We only need these two. All other nodes
        will be explored via the edges.
    """

    # First dictionary is for caching purposes. We don't want to recompute the field value for a given coordinate twice. 
    # Second dictionary maps the coordinates of a point to it's representing node which is used in the actual graph.
    map_koordinates_to_field_value = {}
    map_koordinates_to_node = {}

    # List of all steps we could take in the next iteration. Don't include diagonal steps in the direction. This will
    # result in suboptimal paths being taken in the path planning
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    progress_bar = tqdm(total=width * height, desc="Creating nodes in graph")
    # Create the nodes of the entire graph
    for x in range(0, width + 1):
        for y in range(0, height + 1):
            map_koordinates_to_node[(x, y)] = Node((x, y))
            progress_bar.update(1)

    progress_bar.close()

    progress_bar = tqdm(total=width * height, desc="Adding weights to nodes in graph")
    # Compute edges and store them
    for x in range(0, width + 1):
        for y in range(0, height):
            node_looking_at = map_koordinates_to_node[(x, y)]
            neighboring_points = []

            # Check which neighbouring points we have given a point (x,y). Add these points to a list
            for dx, dy in directions:
                neighbor_x, neighbor_y = x + dx, y + dy

                if 0 < neighbor_x < (width + 1) and 0 < neighbor_y < height + 1:
                    neighboring_points.append((neighbor_x, neighbor_y))

            for point in neighboring_points:
                if point not in map_koordinates_to_field_value:
                    # The potential field value has not been computed yet and not in dictionary. We compute the
                    # value, store it in the dictionary and add the point with the given weight to the neighbour list
                    # of node.
                    intermediate_robot = Robot(point[0], point[1])
                    value = calculate_potential_field_value_temperature(target, obstacles, alpha, temp,
                                                                        intermediate_robot)
                    map_koordinates_to_field_value[point] = value
                    node_looking_at.add_neighbours(map_koordinates_to_node[point], value)
                else:
                    # The potential field value has already been computed. We look it up and then add the node with
                    # the respective weight to the list fo neighbours
                    node_looking_at.add_neighbours(map_koordinates_to_node[point],
                                                   map_koordinates_to_field_value[point])

            progress_bar.update(1)

    progress_bar.close()

    return map_koordinates_to_node[robot.vector], map_koordinates_to_node[target.vector]
