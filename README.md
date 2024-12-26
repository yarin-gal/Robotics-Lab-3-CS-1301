# Robotics-Lab-3-CS-1301
In this lab, my partner and I were tasked with having our iRobot do two things: "Autonomously deliver" itself to a specific location on a coordinate grid while avoiding objects in its path, and a "Maze solver", where the robot has to make intelligent decisions to escape a maze all by itself, without previous knowledge of the layout of the maze. 

# Maze Solver
This project implements an iRobot capable of navigating its way from the start point to the end point of a dynamic, unkown maze. It functions by "building" the maze in real-time by sensing walls and mapping them. The robot first creates a predetermined cost per cell based on an open maze. It then detects a wall using its sensor, maps it, and then recalculates the costs of the cells to reflect the mapped maze. The robot uses a cost-based approach to determine its next move, choosing the cell with the lowest cost. If a tie is found, a predetermined strategy causes it to choose one of the cells. By mapping and recalculating costs of cells, the robot successfully reaches its endpoint.

# Autonomous Delivery
