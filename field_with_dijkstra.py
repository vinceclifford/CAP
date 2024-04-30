from classes.node import Node 
from classes.robot import Robot
from classes.static_circle import Static_Circle
from engine_math import calculate_potential_field_value_temperature
import time 
from neighborhood import dijkstra

def pathplanning_with_potential_field_and_dijkstra(robot, target, obstacles, alpha, temp, width, height): 
    print('We are about to construct the graph')
    start, goal = construct_entire_graph(robot, target, obstacles, width, height)
    path, cost = dijkstra(start, goal)    
    return path 


def construct_entire_graph(robot, target : Static_Circle, obstacles, width, height ,alpha=1, temp=1): 
    # Cache such that we don't recompute values over and over again 
    map_koordinates_to_field_value = {} 
    map_koordinates_to_node = {}
    
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    
    #Create the nodes of the entire graph 
    for x in range(0, width + 1): 
        for y in range(0, height + 1): 
            map_koordinates_to_node[(x,y)] = Node((x,y))
    
    print("Done with creation of the nodes of the enitre graph")

    #Compute edges and store them 
    for x in range(0, width + 1): 
        for y in range(0, height):
            node_looking_at = map_koordinates_to_node[(x,y)] 
            neighboring_points = []
                
            for dx, dy in directions:  
                neighbor_x, neighbor_y = x + dx, y + dy 
                
                if neighbor_x in range(0, width + 1) and neighbor_y in range(0, height + 1): 
                    neighboring_points.append((neighbor_x, neighbor_y))
                    
            for point in neighboring_points:
                if point not in map_koordinates_to_field_value: 
                    intermediate_robot = Robot(point[0], point[1])
                    value = calculate_potential_field_value_temperature(target, obstacles, alpha, temp, intermediate_robot)
                    map_koordinates_to_field_value[point] = value
                    node_looking_at.addNeighbours(map_koordinates_to_node[point], value)
                else: 
                    node_looking_at.addNeighbours(map_koordinates_to_node[point], map_koordinates_to_field_value[point])
                
    print('We added the edges to the graph and are now finished with constructing the entire graph!')

    return map_koordinates_to_node[robot.vektor], map_koordinates_to_node[target.vektor]    