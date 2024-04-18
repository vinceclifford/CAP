class Node: 
    def __init__(self, position, neighbors=None, identifier="") -> None:
        self.position = position
        self.neighbors = neighbors if neighbors is not None else {}
        self.identifier = identifier
        
    def addNeighbours(self, neighbor, weight): 
        self.neighbors[neighbor] = weight 
        
    def addNeghbours(self, dict): 
        self.neighbors.update(dict)
    
        
    def __lt__(self, other):
        return self.position[0] < other.position[1]