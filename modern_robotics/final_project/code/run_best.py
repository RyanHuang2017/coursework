import numpy as np
import modern_robotics as mr
from utilities import *
import matplotlib.pyplot as plt
import pandas as pd

#--------------obtain reference trajectory--------------
# initial and desired cube configuration w.r.t fixed frame {s}
Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
Tsc_desired = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
# gripper standoff config w.r.t cube frame {c} 
theta = 3*np.pi/4
w_hat = np.array([0,1,0])
w = mr.VecToso3(w_hat)
R = mr.MatrixExp3(w*theta)
p = np.array([0,0,0.1])
Tce_standoff = mr.RpToTrans(R,p)
# gripper grasping config w.r.t cube frame {c} 
p = np.array([0,0,0])
Tce_grasp = mr.RpToTrans(R,p)
# intial gripper reference config w.r.t fixed frame {s} for feedback control
Tse_initial_ref = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
refConfigNum = 10
endEfftrRefConfig = TrajectoryGenerator(Tse_initial_ref,Tsc_initial,Tsc_desired,Tce_grasp,Tce_standoff,refConfigNum)
np.savetxt("../results/best/referenceTrajectory.csv",endEfftrRefConfig,delimiter=",")

#--------------feedforward + feedback control setup--------------
# designed initial actual config of the robot (chassis + 5R arm + gripperStatus + moveStage)
currentConfig = np.array([0.5, 0.0, 0.5, 0.0, -np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1])
# initial end effector configuration w.r.t 5R arm base frame {o}
Moe = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
# fixed offset betweeen chassis base and 5R arm base w.r.t chassis base frame {b}
Tbo = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
# body Jacobian of the gripper
B1 = np.array([[0,0,1,0,0.033,0]])
B2 = np.array([[0,-1,0,-0.5076,0,0]])
B3 = np.array([[0,-1,0,-0.3526,0,0]])
B4 = np.array([[0,-1,0,-0.2176,0,0]])
B5 = np.array([[0,0,1,0,0,0]])
Blist = np.concatenate((B1,B2,B3,B4,B5),axis=0).T
# initial actual config of gripper w.r.t the fixed frame {s} and Je w.r.t end effector frame {e}
X,Je = TseJeFromConfig(Blist,Moe,Tbo,currentConfig)
kp,ki = 1.3,0
dt = 0.01/refConfigNum
maxAngularSpeeds = 20
N = np.shape(endEfftrRefConfig)[0]
Kp = kp*np.eye(6)
Ki = ki*np.eye(6)
Eint_prev = np.zeros(6)
Xerr = np.zeros((1,7))
robotConfig = np.zeros((1,13))
robotConfig[0][0:13] = currentConfig.flatten()[0:13]
j = 0
for i in range(N-1):
    # run feedforward & feedback control, update control speed 
    Xd = reconstructTransFromTraj(endEfftrRefConfig[i])
    Xd_next = reconstructTransFromTraj(endEfftrRefConfig[i+1])
    Vet,Xerrt,Eintt = FeedbackControl(X,Xd,Xd_next,Eint_prev,Kp,Ki,dt)
    Xerr = np.concatenate((Xerr,np.insert(Xerrt,0,i*dt).reshape(1,7)),axis=0)
    Je_inv = np.linalg.pinv(Je,rcond = 1e-2)
    control = np.matmul(Je_inv,Vet)

    # update currentConfig, X, Je, Eint_prev for next control loop
    nextConfig = NextState(currentConfig,control,dt,maxAngularSpeeds)
    nextConfig = np.concatenate((nextConfig.flatten(),np.array([endEfftrRefConfig[i+1][12:14]]).flatten()),axis=0).reshape(1,14)
    X,Je = TseJeFromConfig(Blist,Moe,Tbo,nextConfig)
    Eint_prev = Eintt
    currentConfig = nextConfig

    # save animation config and xerr data
    j += 1
    if (j == refConfigNum):
        robotConfig = np.concatenate((robotConfig,np.array([currentConfig[0][0:13]])),axis=0)
        j = 0

Xerr = Xerr[1:]

print("Generating animation csv file.")
np.savetxt("../results/best/animation.csv",robotConfig,delimiter=",") 

xerr = pd.DataFrame(Xerr, columns=['time','wx', 'wy', 'wz','vx','vy','vz'])
print("Visualizing xerr plot data ...")
plt.figure(figsize=(12,6))
plt.title("Xerr vs. Time")
plt.plot(xerr['time'],xerr['wx'],label='wx')
plt.plot(xerr['time'],xerr['wy'],label='wy')
plt.plot(xerr['time'],xerr['wz'],label='wz')
plt.plot(xerr['time'],xerr['vx'],label='vx')
plt.plot(xerr['time'],xerr['vy'],label='vy')
plt.plot(xerr['time'],xerr['vz'],label='vz')
plt.axhline(y = 0,linestyle="--",color = 'k')
plt.text(1.45,-1.5,"stg #1")
plt.axvline(x = 4,linestyle='--',color='k')
plt.text(5.17,-1.5,"stg #2")
plt.axvline(x = 7,linestyle='--',color='k')
plt.text(7.10,-1.5,"stg #3")
plt.axvline(x = 8,linestyle='--',color='k')
plt.text(8.10,-1.5,"stg #4")
plt.axvline(x = 9,linestyle='--',color='k')
plt.text(10.60,-1.5,"stg #5")
plt.axvline(x = 13,linestyle='--',color='k')
plt.text(14.06,-1.5,"stg #6")
plt.axvline(x = 16,linestyle='--',color='k')
plt.text(16.10,-1.5,"stg #7")
plt.axvline(x = 17,linestyle='--',color='k')
plt.text(17.10,-1.5,"stg #8")
plt.ylim([-2,2])
plt.xlim([0,18])
plt.xticks([0,4,7,8,9,13,16,17,18])
plt.ylabel("Xerr")
plt.xlabel("Time")
plt.legend(loc='upper left')
plt.tight_layout()
plt.savefig("../results/best/xerr.png",dpi=600)
# plt.show()
print("Writing xerr plot data.")
np.savetxt("../results/best/xerr.csv",robotConfig,delimiter=",")
print("Done.")