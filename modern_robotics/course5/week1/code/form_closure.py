from scipy.optimize import linprog
import numpy as np 

def form_closure_linprog(contactFile):
    # loading contact information
    contacts = np.loadtxt(contactFile,delimiter=",", comments='#')
    numOfContact = np.shape(contacts)[0]
    
    # for k >= 1
    k = np.ones((1,numOfContact))
    A = np.diag(np.full(numOfContact,-1))
    b = np.full(numOfContact,-1)
    
    # determine the matrix of contact normal wrench
    F = np.zeros((numOfContact,3))
    for i in range(numOfContact):
        pi = contacts[i][0:2]
        theta = np.deg2rad(contacts[i][2])
        R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        ni = np.matmul(R, np.array([[1,0]]).T).T
        Fi = np.append(np.cross(pi,ni),ni).reshape(1,3)
        F[i] = Fi
    F = F.T

    # for Fk = 0
    Aeq = F
    Beq = np.zeros(3)
    res = linprog(k,A,b,Aeq,Beq)
    
    # evaluation of form closure based on linprog results
    if(res.status == 0):
        print("Form closure ACHIEVED.")
        print("the resulting coefficient vector is: ")
        print(res.x)
        print("the resultant Fk is: ")
        print(res.con)
        return 1
    
    print("Form closure FAILED")
    print("the resulting coefficient vector is: ")
    print(res.x)
    print("the resultant Fk is: ")
    print(res.con)
    return 0

# # uncomment to run form closure analysis for case 1
# contactFile = "../results/contacts1.csv"
# form_closure_linprog(contactFile)

# uncomment to run form closure analysis for case 2
contactFile = "../results/contacts2.csv"      
form_closure_linprog(contactFile)