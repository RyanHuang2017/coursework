from modern_robotics import *
import numpy as np
np.set_printoptions(suppress=True)

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot.
    A report for each iteration of the Newton-Raphson process, from the initial
    guess to the final solution, is printed out. The joint vector of each
    iteration is saved as an "iterates.csv" file.

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector in the space frame
    :param T: The desired end-effector configuration Tsd in the space frame
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T)))
    err = (np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg) | (np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev)
    thetalist_iter = thetalist.reshape(1, np.shape(thetalist)[0])

    while (err) & (i < maxiterations):
        
      thetalist = thetalist + np.dot(np.linalg.pinv(JacobianBody(Blist, thetalist)), Vb)
      Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T)))
      err = (np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg) | (np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev)
      thetalist_iter = np.concatenate((thetalist_iter,thetalist.reshape(1,np.shape(thetalist)[0])),axis=0)

      print("Iteration %d: "%i)
      print("joint vector: ")
      print(thetalist)
      print("SE(3) end-effector config: ")
      print(FKinBody(M, Blist, thetalist))
      print("error twist V_b: ")
      print(Vb)
      print("angular error magnitude ||omega_b||: %.6f" %np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
      print("linear error magnitude ||v_b||: %.6f" %np.linalg.norm([Vb[3], Vb[4], Vb[5]]))
      print("\n")
      i+= 1
    
    np.savetxt('../results/iterates.csv',thetalist_iter,delimiter=",")

    return (thetalist, not err)

#EXAMPLE 4.5 @ CH4.1.2
W1,W2,L1,L2,H1,H2=0.109,0.082,0.425,0.392,0.089,0.095
M = np.array([[-1,0,0,L1+L2],[0,0,1,W1+W2],[0,1,0,H1-H2],[0,0,0,1]])
Tsd = np.array([[0,1,0,-0.5],[0,0,-1,0.1],[-1,0,0,0.1],[0,0,0,1]])

# generate the list of screw axis
B1 = np.array([[0,1,0,W1+W2,0,L1+L2]])
B2 = np.array([[0,0,1,H2,-L1-L2,0]])
B3 = np.array([[0,0,1,H2,-L2,0]])
B4 = np.array([[0,0,1,H2,0,0]])
B5 = np.array([[0,-1,0,-W2,0,0]])
B6 = np.array([[0,0,1,0,0,0]])
Blist = np.concatenate((B1,B2,B3,B4,B5,B6),axis=0).T
eomg,ev = 0.001,0.0001
thetalist0 = np.array([-3.64,0.75,-1.5,2.39,0.38,-3.19])
thetalist,err = IKinBodyIterates(Blist,M,Tsd, thetalist0,eomg,ev)
print(thetalist)