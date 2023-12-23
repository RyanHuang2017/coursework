import numpy as np
from enum import Enum
#in this assignment, no collision happens between goal set and obstacles

#kilobot diameter in m
kilobotDiam = 0.033
#newNode search distance in m. value should be larger than the kilobot diameter
searchDist = 0.05
# start node position
startNodePos = np.array([-0.5,-0.5])
# goal node position
goalNodePos = np.array([0.5,0.5])
    
def rrt_planning(obstaclesFile):
    """
    this function takes obtacleFile as input to perform RRT planning, and returns
    lists of nodes and edges resulting from the planning and is_success flag to
    indicates whether the RRT planning is successful.

    Argument:
        obstacleFile --- directory of the obstacleFile
    
    Return:
        is_success --- a True/False flag to indicate whether the RRT planning is successful
        nodes --- a list of nodes resulting from the RRT planning
        edges --- a list of edges resulting from the RRT planning
    """
    # Unidirectional RRT algorithm. the search tree grows from Xstart.
    obstacles = np.loadtxt(obstaclesFile,delimiter=",",comments="#")
    # max number of the nodes resulting from the rrt planning
    maxTreeSize = 1000

    goalSetRange = min_goal_obstacle_dist(goalNodePos,obstacles)
    nodes = []
    edges = []
    is_success = False

    nodes.append([1,startNodePos[0],startNodePos[1],heuristic_cost_to_go(startNodePos,goalNodePos)])
    while(len(nodes) <= maxTreeSize):
        sampleNodePos = np.random.uniform(low=[-0.5,-0.5],high=[0.5,0.5],size=(1,2))
        nearestNodeID, nearestNodePos = find_nearest(sampleNodePos,nodes)
        newNodePos = local_planner(nearestNodePos, sampleNodePos)
        if (collision_checking(obstacles,newNodePos,nearestNodePos)):
            newNodeID = len(nodes) + 1
            nodes.append((newNodeID,newNodePos[0],newNodePos[1],heuristic_cost_to_go(newNodePos,goalNodePos)))
            edges.append((newNodeID,nearestNodeID,np.linalg.norm(newNodePos-nearestNodePos)))
            if (is_goal_reach(newNodePos,goalNodePos,goalSetRange)):
                nodes.append((newNodeID+1,goalNodePos[0],goalNodePos[1],0.0))
                edges.append((newNodeID+1,newNodeID,np.linalg.norm(goalNodePos-newNodePos)))
                is_success = True
                return (is_success,nodes,edges)
    
    return (is_success,nodes,edges)

def min_goal_obstacle_dist(goalNodePos,obstacles):
    """
    this function takes the goal node position and the obstacles info as input
    return the radii of the minimum clear circular area around the goal node.

    Arguments:
        goalNodePos --- position of the goal node
        obstacles   --- array of obstacle info
    Return:
        euclideanDist --- radii of the minimum clear circlar area around the goal node
    """
    euclideanDist = np.inf
    for i in range(len(obstacles)):
        dist = np.linalg.norm(goalNodePos - [obstacles[i][0],obstacles[i][1]]) - 0.5*obstacles[i][2]
        if (euclideanDist > dist):
            euclideanDist = dist
    return euclideanDist

def is_goal_reach(newNodePos,goalNodePos,goalSetRange):
    """
    this function takes the new node position, goal node position and goalSetRange
    values as inputs, and returns whether the new node is witin goal set range

    Arguments:
        newNodePos --- position of the new node
        goalNodePos --- position of the goal node
        goalSetRange --- radii of the minimum clear circular area around the goal node
    Return:
        True/False --- indication of whether the new node is within the goal set range
    """
    dist = np.linalg.norm(newNodePos-goalNodePos) + 0.5*kilobotDiam
    if (dist <= goalSetRange):
            return True
    return False

def find_nearest(currentNodePos, nodes):
    """
    this function takes current node position and array of node info as inputs, and returns
    the id and position of the nearest node in the node array

    Arguments:
        currentNodePos --- position of current node
        nodes --- array of node info
    
    Return: id and position of the node nearest to the current node
    """
    nodeID = 0
    nodePos = np.array([0.0,0.0])
    euclideanDist = np.inf
    for i in range(len(nodes)):
        if (euclideanDist > np.linalg.norm(currentNodePos - [nodes[i][1],nodes[i][2]])):
            euclideanDist = np.linalg.norm(currentNodePos - [nodes[i][1],nodes[i][2]])
            nodeID = nodes[i][0]
            nodePos = np.array([nodes[i][1],nodes[i][2]])

    return nodeID, nodePos

def heuristic_cost_to_go(currentNodePos, goalNodePos):
    """
    this function takes current node position and goal node position as inputs
    and returns the value of heuristic cost to go from current node to goal node
    based on straight-line distance

    Arguments:
        currentNodePos --- position of current node
        goalNodePos --- position of goal node
    
    Return:
        heuristic_cost_to_go value based on the straight-line distance from the current
        node to the goal node
    """
    return np.linalg.norm(currentNodePos - goalNodePos)

def collision_checking(obstacles,newNodePos, nearestNodePos):
    """
    this function takes the array of obstacle info, new node position and nearest node position
    as inputs, and returns true if the new node is conflict with any obstacles, and if the straight
    path from the new node passed through any obstacle.

    Arguments:
        obstacles --- array of obstacle info
        newNodePos --- position of the new node
        nearestNodePos --- position of the nearest node
    
    Return: True if no collision occurs
    """
    lineSegVec1 = np.array([nearestNodePos[0]-newNodePos[0],nearestNodePos[1]-newNodePos[1]])
    for i in range(len(obstacles)):
        if (np.linalg.norm(newNodePos - [obstacles[i][0],obstacles[i][1]]) <= 0.5*(obstacles[i][2] + kilobotDiam)):
            return False
        #vector from nearestNode to the obstacle centroid
        lineSegVec2 = np.array([obstacles[i][0]-nearestNodePos[0],obstacles[i][1]-nearestNodePos[1]])
        #projection of lineSegVec2 along the direction of lineSegVec1
        lineSegVec2Proj = np.dot(lineSegVec1,lineSegVec2)/(np.linalg.norm(lineSegVec1)**2) * lineSegVec1
        #minimal distance from obstacle centroid to the line of lineSegVec1
        minDist = np.linalg.norm(lineSegVec1-lineSegVec2Proj)
        #the size of robot is shrink to zero, while the radii of the obstacle is grown to R+r
        if (minDist > 0.5*(obstacles[i][2]+kilobotDiam)):
            # no intersection
            continue
        else:
            #compare the length of linSegVec2Proj and LineSegVec1
            if (np.linalg.norm(lineSegVec2Proj) > np.linalg.norm(lineSegVec1)):
                #no intersection
                continue
            else:
                #1 intersection when lineSegVec2Proj and lineSegVec1 have the same length
                #2 intersections when lineSegVec2Proj has a length smaller than lineSegVec1
                return False

    return True

def local_planner(nearestNodePos,sampleNodePos):
    """
    This function takes the nearest node position and the sample node position as inputs,
    and returns a new node along the path from the nearest node to the sample node

    Arguments:
        nearestNodePos --- position of the nearest node
        sampleNodePos --- position of the sample node
    
    Return: position of a new node along the path from nearest node to the sample node
    """
    x_near,y_near = nearestNodePos[0], nearestNodePos[1]
    x_samp,y_samp = sampleNodePos[0][0], sampleNodePos[0][1]
    dist = np.linalg.norm([x_samp-x_near,y_samp-y_near])
    x_new = searchDist*(x_samp-x_near)/dist + x_near
    y_new = searchDist*(y_samp-y_near)/dist + y_near
    return np.array([x_new, y_new])

class distributionChoices(Enum):
    uniformDist = 0
    goalConfig = 1

def biased_uniform_distribution():
    p = np.array([0.9,0.10])
    choices = [distributionChoices.uniformDist,distributionChoices.goalConfig]
    x = np.random.choice(choices,1,p)
    if (x == distributionChoices.uniformDist):
        return np.random.uniform(low=[-0.5,-0.5],high=[0.5,0.5],size=(1,2))
    elif (x == distributionChoices.goalConfig):
        return goalNodePos
