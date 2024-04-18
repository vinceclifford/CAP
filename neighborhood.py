import numpy as np 
import heapq 
from engine_math import calculate_potential_field_value_temperature
from classes.robot import Robot   
from classes.node import Node   

def get_minimal_neighboring_path(path, mid_point, radius, target, obstacles, alpha, temp): 
    starting_point, final_point = find_intersecting_points(path, mid_point, radius)
    starting_node, final_node = construct_radius_graph(starting_point, final_point, mid_point, radius, target, obstacles, alpha, temp)
    return dijkstra(starting_node, final_node)
    

def find_intersecting_points(path, midpoint, radius): 
    intersecting_points = []
    
    for point in path: 
        dist = np.linalg.norm(np.array(point) - np.array(midpoint))
        if dist <= radius: 
            intersecting_points.append((point, dist))

    intersecting_points.sort(key= lambda x : x[1])
    return tuple(intersecting_points[:2]) 

    
def dijkstra(start, end): 
    print('Starting with Dijkstras Algorithm')
    distances = {} 
    distances[start] = 0, None
    visited = {start}

    priority_queue = [(0, start)] 
    
    while priority_queue: 
        current_distance, current_node = heapq.heappop(priority_queue)
        print(len(visited))
        
        #Backwards traversal such that we find the path if we reached the end node
        if current_node == end:
            print("Spinning in path construction")
            shortest_path = []
            while current_node is not None:
                
                shortest_path.append(current_node.position)
                print(current_node.position)
                _, parent = distances[current_node]
                current_node = parent
                
            shortest_path.reverse()
            return shortest_path, current_distance   
        
        for neighbor, weight in current_node.neighbors.items(): 
            distance = current_distance + weight 
            
            if neighbor not in visited or distance < distances[neighbor][0]: 
                distances[neighbor] = distance, current_node
                visited.add(neighbor)
                heapq.heappush(priority_queue, (distance, neighbor)) 
                
    return None, float('inf')
    

def construct_radius_graph (starting_point, final_point, mid_point, radius, target, obstacles, alpha, temp): 
    x_start, y_start = starting_point
    x_final, y_final = final_point
    x_mid, y_mid = mid_point
    
    map = {}
    
    # we need to add +1 at the end value is exklusive. If we want to include the point on radius, we need to add one 
    for x in range(mid_point - radius, mid_point + radius + 1): 
        for y in range(mid_point - radius, mid_point + radius + 1): 
        
            # We are evaluating a point outside of the radius that we are interested in, then well continue with the next point 
            if np.linalg.norm(x - x_mid, y - y_mid) > radius: 
                pass 
            
            node_looking_at = None
            if (x,y) in map: 
                node_looking_at = map[(x,y)]
            else: 
                node_looking_at = Node(x,y)
                map[(x,y)] = node_looking_at
            
            neighbouring_points = get_neighboring_points_with_radius(x, y, mid_point, radius)
            
            for point in neighbouring_points: 
                robot = Robot(point[0], point[1])
                node_looking_at.addNeighbours(map[point], calculate_potential_field_value_temperature(target, obstacles, alpha, temp, robot)) 
    
    return (map[(x_start, y_start)], map[(x_final, y_final)])

  
def get_neighboring_points_with_radius(x, y, mid_point, radius):
    points = []
    for i in range (-1, 0, +2): 
        #upper row of circle 
        if np.linalg.norm((x + i - mid_point[0], y + 1 - mid_point[1])) < radius: 
            points.append((x + i, y + 1))
        
        #lower row of circle
        if np.linalg.norm((x + i - mid_point[0], y - 1 - mid_point[1])) < radius: 
            points.append((x + i, y - 1))
            
     #to the left of point 
    if  np.linalg.norm((x + 1 - mid_point[0], y - mid_point[1])) < radius:
        points.append((x + 1, y - 1))
            
    #to the right of point
    if  np.linalg.norm((x - 1 - mid_point[0], y - mid_point[1])) < radius:
        points.append((x - 1, y - 1))
        
    return points 