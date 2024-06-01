# Utilising Potential Fields for Trajectory Planning in Medical Robotics 

In the advancement towards digitizing operating rooms, a fundamental aspect involves the creation of a virtual representation 
of the operating space, enabling seamless navigation for surgical robots within it. In this clinical application project,
I focused on navigating medical robots to the operating position. 

Under the folder `report`, in the document `documentation.pdf` the  motivation of the utilization of potential fields over 
traditional path planning algorithm is explained. Additionally, one might be a bit confused and astonished behind the math
heavy side of this project. The entire maths for this projects is also described in there. 

The most complex math is only done for parallelization by utilizing `PyTorch` mathematical objects called `tensors`. With 
this, incredible, real-time path-planning is achieved which actually can be used in the medical setting. 

# Important Note

This project is in its starting stages. I plan to work on this project for multiple upcoming years. At the moment 
 my focus is doing actual research, exploring new techniques quickly, by rapid prototyping (e.g. misusing PyTorchs' tensors 
for path planning). Therefore, there are still **many** bugs in the code. I plan to fix them eventually. 

# Running the Path Planning Algorithm 

I cleaned up the code to make this part as easy as possible. To run the proposed algorithm which is based on potential 
fields, create your to be tested environment in a file in the `environments` folder. For every environment there must be 
three components: 
* obstacles: A list containing obstacles of either type `StaticCircle` or `StaticPolygon`.
* target: The goal of the path planning algorithm which is a `StaticCircle`.
* agent: Is an instance of the class `Robot` which is just a wrapper class for the starting position of the robot. I 
decided to create a wrapper class because I've got additional features I would like to implement in the future.

In order to run the algorithm on the program, change the import setting in the file `src/main.py` to your newly created 
environment and run the python file. A corresponding heat map should appear as well as the robot's taken path. In the 
future, I will create a more convenient way of running newly created environment than actually changing the source code, by
passing the path of the environment as a program argument. Just did not have time to do this yet.
