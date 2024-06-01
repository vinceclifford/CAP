class Node:
    def __init__(self, position, neighbors=None, identifier="") -> None:
        """
        Args: position ((int, int)): Position that the node represents in the graph neighbors (dict, optional): Map
        that stores all neighbours of node and maps to correspond weight to go to neighbour. Defaults to None.
        identifier (str, optional): Defaults to "".
        """
        self.position = position
        self.neighbors = neighbors if neighbors is not None else {}
        self.identifier = identifier

    def add_neighbours(self, neighbor, weight):
        self.neighbors[neighbor] = weight

    def add_neighbours(self, dict):
        self.neighbors.update(dict)

    def __lt__(self, other):
        """
        Needed for completeness in the heap for dijkstras algorithm. Else we might get an exception
        """
        return self.position[0] < other.position[0]
