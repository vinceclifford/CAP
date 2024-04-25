from classes.node import Node 
from classes.robot import Robot
from classes.static_circle import Static_Circle
from engine_math import calculate_potential_field_value_temperature
from neighborhood import dijkstra

def pathplanning_with_potential_field_and_dijkstra(robot, target, obstacles, alpha, temp, width, height): 
    print('We are about to construct the graph')
    start, goal = construct_entire_graph(robot, target, obstacles, width, height)
    print(len(start.neighbors))
    path, cost = dijkstra(start, goal)
    return path 


def construct_entire_graph(robot, target : Static_Circle, obstacles, width, height ,alpha=1, temp=1): 
    map = {}
    #directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    
    #Create the nodes of the entire graph 
    for x in range(0, width + 1): 
        for y in range(0, height + 1): 
            map[(x,y)] = Node((x,y))
    
    print("Done with creation of the nodes of the enitre graph")

    #Compute edges and store them 
    for x in range(0, width + 1): 
        for y in range(0, height):
            node_looking_at = map[(x,y)] 
            neighboring_points = []
                
            for dx, dy in directions:  
                neighbor_x, neighbor_y = x + dx, y + dy 
                
                if neighbor_x in range(0, width + 1) and neighbor_y in range(0, height + 1): 
                    neighboring_points.append((neighbor_x, neighbor_y))
                    
            for point in neighboring_points:
                intermediate_robot = Robot(point[0], point[1])
                node_looking_at.addNeighbours(map[point], calculate_potential_field_value_temperature(target, obstacles, alpha, temp, intermediate_robot))
                
    print('We added the edges to the graph and are now finished with constructing the entire graph!')

    return map[robot.vektor], map[target.vektor]    