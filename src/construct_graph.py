from src.node import Node


def construct_graph_from_tensor(tensor, start, goal, width, height):
    """

    Args:
        tensor (torch.list): tensor of the potential field values for given environment.
        start (int, int): starting coordinates of robot.
        goal (int, int): goal coordinates / operating table coordinates.
        width (int): width of the environment.
        height (int): height of the environment.

    Returns:
        (node, node): Returns the starting and goal node of the graph. We only need these two. All other nodes
        will be explored via the edges.

    """
    directions = [(1, 0), (-1, 0), (0, -1), (0, 1)]
    map_koordinates_to_node = {}
    for x in range(0, width):
        for y in range(0, height):
            map_koordinates_to_node[(x, y)] = Node((x, y))

    for x in range(0, width):
        for y in range(0, height):
            for direction in directions:
                next_x, next_y = x + direction[0], y + direction[1]
                if 0 <= next_x < width and 0 <= next_y < height:
                    map_koordinates_to_node[(x, y)].add_neighbours(map_koordinates_to_node[(next_x, next_y)],
                                                                   tensor[next_y, next_x].item())

    return map_koordinates_to_node[start], map_koordinates_to_node[goal]
