import matplotlib.pyplot as plt
import sys
import numpy as np
from classes.static_objects import Static_Object
from classes.robot import Robot
from engine_math import calculate_potential_field_value_temperature, calculate_potential_field_value



obstacles = set() 
agent = Robot(-50, 0)
obstacle_1 = Static_Object(50, 0, 100, 3)
obstacle_2 = Static_Object(150, 0, 100, 3)
obstacle_3 = Static_Object(27, 0, 100, 4)
target = Static_Object(100, 0, 1, 5)
obstacles.add(obstacle_1)
obstacles.add(obstacle_2)
obstacles.add(obstacle_3)
x = np.arange(-200,200,0.5)
temps =  [1, 0.6, 0.4, 0.2, 0.1, 0.0001]
y_s = [] 

for temp in temps: 
    y = []
    for x_value in x:
        agent.vektor = (x_value, 0)
        y.append(calculate_potential_field_value_temperature(robot=agent, target=target, obstacles=obstacles, alpha=1, temp=temp))
    y_s.append(y)
    
index = 0 
for temp in temps: 
    plt.plot(x, y_s[index], label = str(temp))
    index += 1

    
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()

plt.show()

    

