from scipy.optimize import linprog
import numpy as np

def stability_evaluation(assemblyFile,contactsFile):

    assembly = np.loadtxt(assemblyFile,delimiter=",", comments='#')
    contacts = np.loadtxt(contactsFile,delimiter=',', comments='#')
    numOfBody = np.shape(assembly)[0]
    numOfContact = np.shape(contacts)[0]
    # c.T*k
    # coefficient of linear objective function c
    c = np.ones((1,2*numOfContact))

    # Fk + Fext = 0
    # determine the contact wrench matrix for body 1
    F = get_contact_wrench(contacts,assembly[0][0])
    
    # determine the contact wrench matrix for the remaining bodies
    if (numOfBody > 1):
        for i in range(1, numOfBody):
            Fi = get_contact_wrench(contacts, assembly[i][0])
            F = np.concatenate((F,Fi),axis=0)


    # for Fk = b
    Aeq = F
    Beq = -1*get_mass_wrench(assembly).T
    # print(Beq)
    res = linprog(c,None,None,Aeq,Beq,bounds=(0,None))
    
    # evaluation of form closure based on linprog results
    if(res.status == 0):
        print("Assembly is Stable.")
        print("the resulting coefficient vector is: ")
        print(res.x)
        print("the resultant Fk + Fext is: ")
        print(res.con)
        return 1
    
    print("Assembly Not Stable")
    # print(res.status)
    print("the resulting coefficient vector is: ")
    print(res.x)
    print("the resultant Fk + Fext is: ")
    print(res.con)
    return 0


def get_contact_wrench(contacts, id):
    numOfContacts = np.shape(contacts)[0]
    F = np.zeros((2*numOfContacts,3))

    indexList,subIndexList= np.where(contacts[:,0:2]==id)
    for i in range(len(indexList)):
        # for each contact, there will be two basis wrenches
        pi = contacts[indexList[i]][2:4]
        miu = contacts[indexList[i]][5]
        frictionAngle = np.arctan(miu)
        thetai_L = np.deg2rad(contacts[indexList[i]][4]) + frictionAngle
        thetai_R = np.deg2rad(contacts[indexList[i]][4]) - frictionAngle
        R_L = np.array([[np.cos(thetai_L),-np.sin(thetai_L)],[np.sin(thetai_L),np.cos(thetai_L)]])
        R_R = np.array([[np.cos(thetai_R),-np.sin(thetai_R)],[np.sin(thetai_R),np.cos(thetai_R)]])
        fi_L = np.matmul(R_L, np.array([[1,0]]).T).T
        fi_R = np.matmul(R_R, np.array([[1,0]]).T).T
        Fi_L = np.append(np.cross(pi,fi_L),fi_L).reshape(1,3)
        Fi_R = np.append(np.cross(pi,fi_R),fi_R).reshape(1,3)

        F[2*indexList[i]] = Fi_L
        F[2*indexList[i]+1] = Fi_R
        if (subIndexList[i] == 1):
            F[2*indexList[i]] = -Fi_L
            F[2*indexList[i]+1] = -Fi_R

    return F.T

def get_mass_wrench(assembly):
    numOfBody = np.shape(assembly)[0]
    Fext = np.zeros((1,numOfBody*3))
    for i in range(numOfBody):
        ra = assembly[i][1:3]
        fa = np.array([0,-assembly[i][3]*9.81])
        ma = np.cross(ra,fa)
        Fexti = np.array([[ma,fa[0],fa[1]]])
        Fext[0][3*i:3*i+3] = Fexti
    return Fext

# # uncomment to run stability evaluation for the collapsing assembly
# contactsFile = "../results/contacts1.csv"
# assemblyFile = "../results/assembly1.csv"
# stability_evaluation(assemblyFile,contactsFile)

# # uncomment to run stability evaluation for the standing assembly
# contactsFile = "../results/contacts2.csv"
# assemblyFile = "../results/assembly2.csv"
# stability_evaluation(assemblyFile,contactsFile)

# uncomment to run stability evaluation for the self-designed collapsing assembly
contactsFile = "../results/contacts3.csv"
assemblyFile = "../results/assembly3.csv"
stability_evaluation(assemblyFile,contactsFile)

# # uncomment to run stability evaluation for the self-designed standing assembly
# contactsFile = "../results/contacts4.csv"
# assemblyFile = "../results/assembly4.csv"
# stability_evaluation(assemblyFile,contactsFile)