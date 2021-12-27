import modern_robotics as mr
import numpy as np

## TrajectoryGenerator() can be called to generate the trajectory of the robot end-effector by using the commented code below.
## The code below also saves the trajectory as a CSV for CoppeliaSim to simulate. Note that TrajectoryGenerator() must be defined
## prior to calling it, so one must cut and paste the commented code below def TrajectoryGenerator() in order to run it.
'''
d1min = 2/100 #cm
d1max = 7/100 #cm
d2 = 2/100 #cm
d3 = 4.3/100 #cm
cube_w = 5/100 #cm

Tseinit = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]
Tscinit = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
Tscfinal = [[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]]
Tcestandoff = [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, d2+cube_w], [0, 0, 0, 1]] #maybe make taller/shorter
Tcegrasp = [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, -d2], [0, 0, 0, 1]] #maybe make taller/shorter
k = 1

traj = TrajectoryGenerator(Tseinit, Tscinit, Tscfinal, Tcegrasp, Tcestandoff, k)
#save joint vectors in a CSV with each vector as its own row
import csv
with open("D:/Northwestern/ME449/Final_Project/Yen_Bill_milestone2/checkpoint2.csv", 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(traj)
'''
def extract_R(T):
    #this takes T in the form
    # [[1.   0.   0.   1.  ]
    # [0.   1.   0.   0.  ]
    # [0.   0.   1.   0.11]
    # [0.   0.   0.   1.  ]]
    # and returns the R portion of the matrix as the vector [1, 0, 0, 0, 1, 0, 0, 0, 1]
    Rtemp = []
    for row in T:
        Rtemp.append(row[0:3]) #take first 3 element of each row
    R = [item for sublist in Rtemp[:-1] for item in sublist] #combine subvectors into one vector
    return R

def extract_P(T):
    #this takes T in the form
    # [[1.   0.   0.   1.  ]
    # [0.   1.   0.   0.  ]
    # [0.   0.   1.   0.11]
    # [0.   0.   0.   1.  ]]
    # and returns the P portion of the matrix as the vector [1, 0, 0.11]
    P = []
    for row in T:
        P.append(row[-1])
    return P[:-1]

def TrajectoryGenerator(Tseinit, Tscinit, Tscfinal, Tcegrasp, Tcestandoff, k):
    # Step 1: Move EE to standoff pos above cube
    Tf = 10
    Xend1 = np.matmul(Tscinit, Tcestandoff)
    traj1 = mr.CartesianTrajectory(Tseinit,Xend1,Tf,Tf*k/0.01,3)
    # Step 2: Move EE down to grasp position
    Xend2 = np.matmul(Tscinit, Tcegrasp)
    traj2 = mr.CartesianTrajectory(Xend1,Xend2,Tf,Tf*k/0.01,3)
    # Step 3: Close gripper (change grip state to 1)
    traj3 = []
    for ii in range(63):
        traj3.append(traj2[-1])
    # Step 4: Move EE back to standoff configuration
    Xend4 = np.matmul(Tscinit, Tcestandoff)
    traj4 = mr.CartesianTrajectory(Xend2,Xend4,Tf,Tf*k/0.01,3)
    # Step 5: Move EE to new standoff configuration above final configuration
    Xend5 = np.matmul(Tscfinal, Tcestandoff)
    traj5 = mr.CartesianTrajectory(Xend4,Xend5,Tf,Tf*k/0.01,3)
    traj5[-1]
    # Step 6: Move from standoff pos down to final grasp configuration
    Xend6 = np.matmul(Tscfinal, Tcegrasp)
    traj6 = mr.CartesianTrajectory(Xend5,Xend6,Tf,Tf*k/0.01,3)
    # Step 7: Opening the gripper
    traj7 = []
    for ii in range(63):
        traj7.append(traj6[-1])
    # Step 8: Move from final position to standoff pos
    Xend8 = np.matmul(Tscfinal, Tcestandoff)
    traj8 = mr.CartesianTrajectory(Xend6,Xend8,Tf,Tf*k/0.01,3)

    # Add all 8 trajectories together
    total_traj = traj1 + traj2 + traj3 + traj4 + traj5 + traj6 + traj7 + traj8
    result = []
    for ii in range(len(total_traj)):
        R = extract_R(total_traj[ii])
        P = extract_P(total_traj[ii])
        if ii <= 2*Tf*k/0.01:
            gripperstate = 0
        if ii > 2*Tf*k/0.01 and ii <= 5*Tf*k/0.01 + 63:
            gripperstate = 1
        if ii > 5*Tf*k/0.01 + 63:
            gripperstate = 0
        line_with_subvectors = [R, P, [gripperstate]]
        line = [item for sublist in line_with_subvectors for item in sublist]
        result.append(line)
    # RETURN: rep of the N configurations of the ee along the entire concatenated eight-segment reference
    # geometry
    return result

'''
d1min = 2/100 #cm
d1max = 7/100 #cm
d2 = 2/100 #cm
d3 = 4.3/100 #cm
cube_w = 5/100 #cm

Tseinit = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]
Tscinit = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
Tscfinal = [[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]]

theta_y = 4*np.pi/6
Tcestandoff = [[np.cos(theta_y), 0, np.sin(theta_y), 0], [0, 1, 0, 0], [-np.sin(theta_y), 0, np.cos(theta_y), d2+cube_w], [0, 0, 0, 1]]
Tcegrasp = [[np.cos(theta_y), 0, np.sin(theta_y), 0], [0, 1, 0, 0], [-np.sin(theta_y), 0, np.cos(theta_y), 0], [0, 0, 0, 1]]

#Tcestandoff = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, d2+cube_w], [0, 0, 0, 1]] #maybe make taller/shorter
#Tcegrasp = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]] #maybe make taller/shorter

#Tcestandoff = [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, d2+cube_w], [0, 0, 0, 1]] #maybe make taller/shorter
#Tcegrasp = [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, -d2], [0, 0, 0, 1]] #maybe make taller/shorter
k = 1

traj = TrajectoryGenerator(Tseinit, Tscinit, Tscfinal, Tcegrasp, Tcestandoff, k)
#save joint vectors in a CSV with each vector as its own row
import csv
with open("D:/Northwestern/ME449/Final_Project/checkpoint2.csv", 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(traj)
'''