import numpy as np
# by default, start node is node of ID 1, goal node is node of ID n

def heuristic_cost_to_go(currentNode,goalNode, nodesArray):
    """
    This function takes the id of two nodes and the info array of all
    nodes as inputs, and returns the euclidean distance between the 
    two nodes as the heuristic cost to go value for the A* search
    Algorithm.
    Argument:
        currentNode:   ID of the current node
        goalNode:      ID of the goal node
        nodeArray:     The info array of all the considered nodes
    Return:
        euclideanDist: Euclidean distance or straight line 
                       distance between current node and goal node
    """
    index_curr = np.where(nodesArray==currentNode)[0][0]
    index_goal = np.where(nodesArray==goalNode)[0][0]
    x_curr, y_curr = nodesArray[index_curr][1],nodesArray[index_curr][2]
    x_goal, y_goal = nodesArray[index_goal][1],nodesArray[index_goal][2]
    euclideanDist = np.linalg.norm([x_curr-x_goal, y_curr-y_goal])
    return euclideanDist

def get_path(currentNode, parentArray):
    """
    This function takes the id of current node and the parent array
    as inputs, and returns path from current node to start node.
    Argument:
        currentNode:   ID of the current node
        parentArray:   The info array of all the considered nodes
    Return:
        path:          a list including id of nodes along the path
    """
    path = []
    path.append(currentNode)
    while (currentNode > 1):

        if (currentNode == 1):
            path.reverse()
            return path
    
        parentNode = parentArray[int(currentNode - 1)]
        path.append(int(parentNode))
        currentNode = parentNode
    
    path.reverse()
    return path

def find_nbr(currentNode, edgesArray):
    """
    This function takes the id of current node and the info array
    of edges as inputs, and returns neighbor nodes of current node.
    Argument:
        currentNode:   ID of the current node
        edgesArray:    The info array of all edges
    Return:
        nbrList:       A list of neighbor node id for current node.
    """
    nbrList = []
    index = np.where(edgesArray==currentNode)[0]
    if (len(index)==0):
        return nbrList
    
    for i in range (len(index)):
        for j in range (2):
            if (edgesArray[index[i]][j] != currentNode):
                nbrList.append(int(edgesArray[index[i]][j]))

    return nbrList

def sort_OPEN_list(node, estTotalCost, openList):
    """
    This function takes node id, estTotalCost and openList 
    as inputs, and returns the sorted openList after append the
    tuple of (node, estTotalCost) to openList.
    Argument:
        node:            ID of the current node
        estTotalCost:    The info array of all edges
    Return:
        openList:        Sorted OPEN list containing nodes to be search.
    """

    if (len(openList)==0):
        openList.append((node,estTotalCost))
        return openList
    
    openList.append((node,estTotalCost))
    openList = sorted(openList, key=lambda nodeTuple: nodeTuple[1])
    
    return openList

def A_start(nodesFile,edgesFile):
    #load info of nodes and edges
    nodes = np.loadtxt(nodesFile,delimiter=",", comments='#')
    edges = np.loadtxt(edgesFile,delimiter=",", comments='#')
    n = np.shape(nodes)[0]          # number of nodes
    m = np.shape(edges)[0]          # number of edges
    startNode = int(nodes[0][0])    # start node id
    goalNode = int(nodes[n-1][0])   # goal node id

    #INITIALIZATION
    openList = []
    closedList = []

    cost = np.zeros((n,n)) - 1
    for i in range(m):
        cost[int(edges[i][0]-1),int(edges[i][1]-1)] = edges[i][2]
        cost[int(edges[i][1]-1),int(edges[i][0]-1)] = edges[i][2]
    
    past_cost = np.zeros(n)
    parent = np.zeros(n)

    is_succeed = False
    past_cost[startNode - 1] = 0
    past_cost[startNode:] = np.inf
    openList.append((startNode,past_cost[startNode - 1] + heuristic_cost_to_go(startNode,goalNode,nodes)))

    while (len(openList)!= 0):
        current = openList.pop(0)
        currentNode = int(current[0])
        closedList.append(currentNode)

        if (currentNode == goalNode):
            is_succeed = True
            return (is_succeed, get_path(currentNode,parent))

        nbrList = find_nbr(currentNode, edges)
        for i in range(len(nbrList)):
            tentative_past_cost = 0.0
            if (nbrList[i] not in closedList):
                tentative_past_cost = past_cost[currentNode - 1] + cost[currentNode - 1, nbrList[i] - 1]
                if (tentative_past_cost < past_cost[nbrList[i]-1]):
                    past_cost[nbrList[i] - 1] = tentative_past_cost
                    parent[nbrList[i] - 1] = currentNode
                    est_total_cost = past_cost[nbrList[i] - 1] + heuristic_cost_to_go(nbrList[i],goalNode, nodes)
                    openList = sort_OPEN_list(nbrList[i],est_total_cost, openList)   

    return (is_succeed, get_path(currentNode,parent))

