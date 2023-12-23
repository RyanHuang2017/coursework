import numpy as np
import modern_robotics as mr

l,w,r = 0.47/2,0.3/2,0.0475
# Mile Stone 1
def NextState(currentConfig,control,timestep,maxAngularSpeeds):
    """
    Arguments:
        currentConfig --- current robot config (3*chassis config,5*joint angles,4*wheel angles, 1*gripperStatus)
        control --- control of wheel speeds and joint speeds(4*wheels,5*joints)
        timestep --- small duration for moving control
        maxAngularSpeeds --- maximum angular speeds of the wheels
    Return:
        nextConfig --- robot configuraton after timestep
    """
    
    #---Current Configuration Input---
    # make sure the currentConfig is a 1D array for processing
    currentConfig = currentConfig.flatten() 
    q = currentConfig[0:3]
    thetaList_arm = currentConfig[3:8]
    thetaList_wheel = currentConfig[8:12]
    
    #--- Chassis Control ---
    control = control.flatten()
    u = control[0:4]
    uIndexList = np.where(np.abs(u)>maxAngularSpeeds)
    if (len(uIndexList[0])):
        u[uIndexList[0]]=np.sign(u[uIndexList[0]])*maxAngularSpeeds
        
    dTheta_wheel = timestep*u
    thetaList_wheel_ = thetaList_wheel + dTheta_wheel
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],[1,1,1,1],[-1,1,-1,1]])
    Vb = np.matmul(F,dTheta_wheel.reshape(1,4).T)
    
    # change of chassis coordinates relative to chassis body frame
    dqb = np.array([[0,Vb[1][0],Vb[2][0]]]).T
    if (Vb[0][0]!= 0.0):
        dqb = np.array([Vb[0][0],
                        (Vb[1][0]*np.sin(Vb[0][0])+Vb[2][0]*(np.cos(Vb[0][0])-1))/Vb[0][0],
                        (Vb[2][0]*np.sin(Vb[0][0])+Vb[1][0]*(1-np.cos(Vb[0][0])))/Vb[0][0]])

    # change of chassis coordinates relative to fixed frame {s}
    dqs = np.matmul(np.array([[1,0,0],[0,np.cos(q[0]),-np.sin(q[0])],[0,np.sin(q[0]),np.cos(q[0])]]),dqb.reshape(1,3).T)
    
    # updated chassis configuration in fixed frame
    q_ = q.reshape(1,3).T + dqs

    #--- 5R Arm Control ---
    speedFactor = 0.8
    jointVelocity = control[4:9]
    jIndexList = np.where(np.abs(jointVelocity)>maxAngularSpeeds*speedFactor)
    if (len(jIndexList[0])):
        jointVelocity[jIndexList[0]]=np.sign(jointVelocity[jIndexList[0]])*maxAngularSpeeds*speedFactor
    thetaList_arm_ = thetaList_arm + jointVelocity*timestep

    #---Next Configuration Output---
    nextConfig = np.zeros((1,12))
    nextConfig = np.concatenate((q_.reshape(1,3),thetaList_arm_.reshape(1,5)),axis=1)
    nextConfig = np.concatenate((nextConfig,thetaList_wheel_.reshape(1,4)),axis=1)

    return nextConfig

 #   Mile Stone 2
def TrajectoryGenerator(initEndEfftrConfig,initCubeConfig,goalCubeConfig,endEfftrConfigGrasp,endEfftrConfigStandoff,refConfigNum):
    """
    This function uses the CartesianTrajectory function to generate a reference trajectory for the end effecor.
    the trajectory consist of 8 move stages:
        moveStage 1: move gripper from initial to standoff position above the cube
        moveStage 2: move gripper from standoff position to grasping position
        moveStage 3: grasping the cube at the initial cube position
        moveStage 4: move gripper from grasping position back to standoff position
        moveStage 5: transfer the cube from initial standoff position to goal standoff position
        moveStage 6: move gripper from goal standoff position to goal grasping position
        moveStage 7: release the cube to the goal cube position
        moveStage 8: move gripper from grasping position back to standoff position

    Arguments:
        initEndEfftrConfig --- initial end effector configuration w.r.t the fixed frame {s}
        initCubeConfig --- initial cube configuration w.r.t the fixed frame {s}
        goalCubeConfig --- desired cube configuration w.r.t the fixed frame {s}
        endEfftrConfigGrasp --- end effector configuration w.r.t the cube frame when grasping
        endEfftrConfigStandoff --- end effector configuration w.r.t the cube frame when standoff (before and after grasping)
        refConfigNum --- number of reference configuration per 0.01 seconds
    
    Return:
        endEfftrRefConfig --- an array of end effector configurations along the whole reference trajectory
    """

    method = 5 # quintic time scaling
    
    # moveStage 1: 
    Tf = 4
    N = Tf/0.01*refConfigNum
    Xstart = initEndEfftrConfig
    Xend = np.matmul(initCubeConfig,endEfftrConfigStandoff)
    endEfftrRefConfig = np.zeros((int(N),14))
    traj = mr.CartesianTrajectory(Xstart,Xend,Tf,N,method)
    graspState,moveStage = 0,1
    for i in range(int(N)):
        [Ri,pi] = mr.TransToRp(traj[i])
        temp = np.concatenate((Ri.flatten(),pi.flatten(),np.array([graspState]),np.array([moveStage])),axis=0).reshape(1,14)
        endEfftrRefConfig[i] = temp

    # moveStage 2: 
    Tf = 3
    N = Tf/0.01*refConfigNum
    Xstart = reconstructTransFromTraj(endEfftrRefConfig[-1])
    Xend = np.matmul(initCubeConfig,endEfftrConfigGrasp)
    traj = mr.CartesianTrajectory(Xstart,Xend,Tf,N,method)
    graspState,moveStage = 0,2
    for i in range(int(N)):
        [Ri,pi] = mr.TransToRp(traj[i])
        temp = np.concatenate((Ri.flatten(),pi.flatten(),np.array([graspState]),np.array([moveStage])),axis=0).reshape(1,14)
        endEfftrRefConfig = np.concatenate((endEfftrRefConfig,temp),axis=0)
    
    # moveStage 3: 
    Tf = 1
    N = Tf/0.01*refConfigNum
    Xend = reconstructTransFromTraj(endEfftrRefConfig[-1])
    graspState,moveStage = 1,3
    [Ri,pi] = mr.TransToRp(Xend)
    for i in range(int(N)):
        temp = np.concatenate((Ri.flatten(),pi.flatten(),np.array([graspState]),np.array([moveStage])),axis=0).reshape(1,14)
        endEfftrRefConfig = np.concatenate((endEfftrRefConfig,temp),axis=0)

    # moveStage 4: 
    Tf = 1
    N = Tf/0.01*refConfigNum
    Xstart = reconstructTransFromTraj(endEfftrRefConfig[-1])
    Xend = np.matmul(initCubeConfig,endEfftrConfigStandoff)
    traj = mr.CartesianTrajectory(Xstart,Xend,Tf,N,method)
    graspState,moveStage = 1,4
    for i in range(int(N)):
        [Ri,pi] = mr.TransToRp(traj[i])
        temp = np.concatenate((Ri.flatten(),pi.flatten(),np.array([graspState]),np.array([moveStage])),axis=0).reshape(1,14)
        endEfftrRefConfig = np.concatenate((endEfftrRefConfig,temp),axis=0)

    # moveStage 5: 
    Tf = 4
    N = Tf/0.01*refConfigNum
    Xstart = reconstructTransFromTraj(endEfftrRefConfig[-1])
    Xend = np.matmul(goalCubeConfig,endEfftrConfigStandoff)
    traj = mr.CartesianTrajectory(Xstart,Xend,Tf,N,method)
    graspState,moveStage = 1,5
    for i in range(int(N)):
        [Ri,pi] = mr.TransToRp(traj[i])
        temp = np.concatenate((Ri.flatten(),pi.flatten(),np.array([graspState]),np.array([moveStage])),axis=0).reshape(1,14)
        endEfftrRefConfig = np.concatenate((endEfftrRefConfig,temp),axis=0)

    # moveStage 6: 
    Tf = 3
    N = Tf/0.01*refConfigNum
    Xstart = reconstructTransFromTraj(endEfftrRefConfig[-1])
    Xend = np.matmul(goalCubeConfig,endEfftrConfigGrasp)
    traj = mr.CartesianTrajectory(Xstart,Xend,Tf,N,method)
    graspState,moveStage = 1,6
    for i in range(int(N)):
        [Ri,pi] = mr.TransToRp(traj[i])
        temp = np.concatenate((Ri.flatten(),pi.flatten(),np.array([graspState]),np.array([moveStage])),axis=0).reshape(1,14)
        endEfftrRefConfig = np.concatenate((endEfftrRefConfig,temp),axis=0)

    # moveStage 7: 
    Tf = 1
    N = Tf/0.01*refConfigNum
    Xend = reconstructTransFromTraj(endEfftrRefConfig[-1])
    graspState,moveStage = 0,7
    [Ri,pi] = mr.TransToRp(Xend)
    for i in range(int(N)):
        temp = np.concatenate((Ri.flatten(),pi.flatten(),np.array([graspState]),np.array([moveStage])),axis=0).reshape(1,14)
        endEfftrRefConfig = np.concatenate((endEfftrRefConfig,temp),axis=0)
    
    # moveStage 8: 
    Tf = 1
    N = Tf/0.01*refConfigNum
    Xstart = reconstructTransFromTraj(endEfftrRefConfig[-1])
    Xend = np.matmul(goalCubeConfig,endEfftrConfigStandoff)
    traj = mr.CartesianTrajectory(Xstart,Xend,Tf,N,method)
    graspState,moveStage = 0,8
    for i in range(int(N)):
        [Ri,pi] = mr.TransToRp(traj[i])
        temp = np.concatenate((Ri.flatten(),pi.flatten(),np.array([graspState]),np.array([moveStage])),axis=0).reshape(1,14)
        endEfftrRefConfig = np.concatenate((endEfftrRefConfig,temp),axis=0)

    return endEfftrRefConfig

# Mile Stone 3  
def FeedbackControl(X,Xd,Xd_next,Eint_prev,Kp,Ki,dt):
    """
    this function implements feedforward and feedback control algorithm to realize robot control.

    Argument:
        X --- end effector configuration w.r.t the fixed frame {s}
        Xd --- end effector reference configuration w.r.t the fixed frame {s}
        Xd_next --- end effector reference configuration after dt w.r.t the fixed frame {s}
        Eint_prev --- integral error accumulated along the feedback control loop before the control
        Kp,Ki --- gain matrixes for the PI control
        dt --- time interval between any two reference configurations
    
    Return:
        Vt --- robot velocity vector resulting from the control
        Xerr --- error matrix between X and Xd
        Eint --- integral error accumulated after the control
    """
    # compute feedforward twist
    Vdt = mr.se3ToVec((1/dt) * mr.MatrixLog6(np.matmul(mr.TransInv(Xd),Xd_next)))
    # express Vd w.r.t the current end effector frame {e} at X
    Vedt = np.dot(mr.Adjoint(np.matmul(mr.TransInv(X),Xd)),Vdt)
    # compute Xerr between X and Xd
    Xerr = mr.se3ToVec(mr.MatrixLog6(np.matmul(mr.TransInv(X),Xd)))
    # compute propotional error
    Eprop = np.matmul(Kp,Xerr)
    # compute integral error
    Eint = Eint_prev + np.matmul(Ki,Xerr*dt)
    # compute end effector twist Vt in the task space
    Vt = Vedt + Eprop + Eint
    return Vt, Xerr, Eint

def reconstructTransFromTraj(endEfftrConfig):
    """
    this function extracts the end effector configuration from the given end effector config.

    """
    endEfftrConfig = endEfftrConfig.flatten()
    R = endEfftrConfig[0:9].reshape(3,3)
    p = endEfftrConfig[9:12].reshape(3,1)
    T = mr.RpToTrans(R,p)
    return T

def TseJeFromConfig(Blist,Moe,Tbo,robotConfig):
    """
    this function extracts the mobile manipulator Jacobian w.r.t to the end effector 
    frame {e} and the end effector configuration w.r.t the fixed frame {s}.

    Arguments:
        Blist --- list of screw axis of the arm joints w.r.t the end effector frame {e}
        Moe --- end effecotor home configuration w.r.t the fixed frame {s}
        Tbo --- fixed offset between the chassis base frame {b} and the arm base frame {o}
        robotConfig --- robot config (3*chassis, 5*joints,4*wheel speeds)

    Return:
        Tse --- end effecor configuration w.r.t the fixed frame {s}
        Je --- mobile manipulator Jacobian
    """
    robotConfig = robotConfig.flatten()
    moveStage = robotConfig[13]
    phi,x,y = robotConfig[0],robotConfig[1],robotConfig[2]
    Tsb = np.array([[np.cos(phi),-np.sin(phi),0,x],[np.sin(phi),np.cos(phi),0,y],[0,0,1,0.0963],[0,0,0,1]])
    thetaList = robotConfig[3:8]
    Toe = mr.FKinBody(Moe,Blist,thetaList)
    # compute the end effector configuration w.r.t the fixed frame {s}
    Tse = np.matmul(np.matmul(Tsb,Tbo),Toe)
    # compute the body Jacobian of the end effector
    Jarm = mr.JacobianBody(Blist,thetaList)
    
    limitReached = checkJointLimits(thetaList)
    if (1 in limitReached[:]):
        index = np.where(limitReached == 1)[0]
        Jarm[:][index] = 0
        # print(index)
        # print(Jarm)
        # print('\n')

    # compute the Jacobian of chassis base w.r.t the end effector frame {e}
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],[1,1,1,1],[-1,1,-1,1]])
    F6 = np.zeros((6,4))
    F6[2:5][:] = F
    Jbase = np.matmul(mr.Adjoint(np.matmul(mr.TransInv(Toe),mr.TransInv(Tbo))),F6)
    if (moveStage!=1 and moveStage!=5):
        Jbase = np.zeros_like(Jbase)
    # mobile manipulator Jacobian w.r.t the end effector frame {e}
    Je = np.concatenate((Jbase,Jarm),axis=1)
    return Tse, Je

def checkJointLimits(jointThetaList):
    """
    This function determines whether the 5R arm joints reach their limits
    based on the given robot configurations.
    For this project, the following joint limits are considered:
    joint 2: [-1.2,1]
    joint 3: [-2.5,0]
    joint 4: [-1.2,0]
    """
    thetaList = jointThetaList.flatten()
    j2lim = np.array([-2,3])
    j3lim = np.array([-2,3])
    j4lim = np.array([-2,3])

    limitCheck = np.array([0,0,0,0,0])
    if (thetaList[1]<j2lim[0] or thetaList[1]>j2lim[1]):
        limitCheck[1] = 1
    
    if (thetaList[2]<j3lim[0] or thetaList[2]>j3lim[1]):
        limitCheck[2] = 1
    
    if (thetaList[3]<j4lim[0] or thetaList[3]>j4lim[1]):
        limitCheck[3] = 1
    
    return limitCheck