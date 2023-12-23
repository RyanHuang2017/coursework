from rrt import *
from a_start_search import *

resultPath = "../results/"
obstaclesFile = "../results/obstacles.csv"
is_success,nodes,edges=rrt_planning(obstaclesFile)
print("\n")
print("Is RRT planning successful?")
print(is_success)
if(is_success):
    print("total number of nodes is: ")
    print(len(nodes))
    np.savetxt(resultPath+"nodes.csv",nodes,delimiter=",")
    print("total number of edges is: ")
    print(len(edges))
    np.savetxt(resultPath+"edges.csv",edges,delimiter=",")
    # run a star search to generate the path
    nodesFile = resultPath + "nodes.csv"
    edgesFile = resultPath + "edges.csv"
    is_succeed, path = A_start(nodesFile,edgesFile)
    np.array(path).tofile(resultPath+"path.csv",sep=',')