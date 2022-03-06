Project 2
Author: Justin Cheng
Class: ENPM 662 - Dr. Reza Monfaredi

Task
----
Checks the feasibility of all inputs/outputs (user provides start and goal nodes that are in the obstacle space). Implements Dijkstra’s Algorithm to find a path between start and end point on a given map for a point robot (radius = 0; clearance = 5 mm). Outputs an animation of node exploration and optimal path between start and goal points on the map.

Used Libraries
--------------
cv2, numpy, collections (deque)

Steps to run code
-----------------
1. Ensure that you have Python 3 installed
	a. Code was written using Python 3.7
2. Extract zip file into any directory
3. Open Dijkstra-pathplanning-Justin-Cheng.py
	a. Opening with Spyder Version 4 or higher is recommended (used to write code)
4. Run Dijkstra-pathplanning-Justin-Cheng.py
	a. Running from terminal or from IDE should both work
	b. Ensure that openCV (cv2) is accessible in the enviornment
5. Note: Try to keep the starting and goal coordinates within 50 pixels of each other, or run times could get very long.
	a. With start node (100,200) and goal node (300, 200), obtained runtime of about 41 minutes (38 minutes to find goal, 3 minutes to visualize)
	b. Try to keep tested nodes within 75 pixels of each other for runtimes below 10 minutes
