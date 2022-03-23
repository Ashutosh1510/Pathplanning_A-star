# **Implementation of A-star Algorithm for a Mobile Robot**
The goal of the project is to implement A* algorithm to find a path between start and end point on a given map for a mobile robot.
## A* Algorithm:
This Algorithm is the advanced form of the BFS algorithm (Breadth-first search), which searches for the shorter path by finding the least cost from the starting point to the ending point. It is a complete as well as an optimal solution for solving path and grid problems.

# Dependencies
1. Python3
2. Opencv
3. Numpy
4. Matplotlib
5. argparse

## Steps to Run the code
- Git clone:
```
git clone https://github.com/Ashutosh1510/Pathplanning_A-star
```
1. Change directory to the root of the directory
```
cd Pathplanning_A-star-main/A_star/source
```
2. To run the code
```
python3 main.py
```
### **Parameters**
- radius: Radius of the robot
- clearance: Clearance for the Obstacle
- theta_Angle: Orientation of the robot at each node
- Xs: X-coordinate of the starting point of the robot
- Ys: y-coordinate of the starting point of the robot
- Xg: X-coordinate of the goal point of the robot
- Yg: y-coordinate of the goal point of the robot

# Example Figure after backtracking

![Figure_1](https://user-images.githubusercontent.com/76493296/159763590-2f71888c-22d8-4330-809d-35416378cc71.png)

