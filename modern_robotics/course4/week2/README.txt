In this project, the RRT algorithm was implemented for the path planning task.

For each iteration, one node was sampled from the uniformly distributed space; 
The nearest node to the sampled node was identified based on whether the 
Euclidean distance in between is minimal. a NEW node was then introduced along
the direction from the nearest node to the sampled node using a straight-line 
local planner and a small search distance. 

The minimal distance from the goal
node to the surrounding obstacles is used to evaluate whether the new node is
in the goal set. Specifically, if the distance from the new node to the goal node
is smaller than the minimal distance between the goal node and the surrounding
obstacles, the new node is within goal set, and the search process is completed.

Once the search is completed, the A* search algorithm is implemented to find the
optimal patht from the start node to the goal node.

Some parameters used in the study:
1. kilobot diameter: 0.033 m
2. search distance : 0.050 m