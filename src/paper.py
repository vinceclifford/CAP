import matplotlib.pyplot as plt

from math_engine import calculate_single_repulsion, calculate_attraction, calculate_potential_field_value_temperature
from classes.static_circle import Static_Circle
from classes.robot import Robot

c1 = Static_Circle(-50, 0, 100, 3, no_interference=0)
c2 = Static_Circle(40, 0, 100, 3, no_interference=0)


list_rep = []
list_att = []
for x in range(-200, 200): 
    robot = Robot(x, 0)
    
    if x == 40: 
        list_rep.append(list_rep[-1])
    else: 
        list_rep.append(calculate_single_repulsion(robot, c2))
        
    list_att.append(calculate_attraction(robot, c1))
      
list_of_values = [t[0] + t[1] for t in zip(list_rep, list_att)]
        
    
    
plt.plot(range(-200,200), list_of_values)
plt.xlabel("X-Coordinate")
plt.ylabel("Total Force Value")
plt.title("Plot")

plt.show()