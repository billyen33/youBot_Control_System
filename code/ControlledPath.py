import modern_robotics as mr
from math import tan, cos, sin, pi
import numpy as np

from Checkpoint1 import NextState
from Checkpoint2 import TrajectoryGenerator
from Checkpoint3 import FeedbackControl, getSpeeds, getJe
from JointLimit import testJointLimits

# Initialize parameters we want to keep constant
d1min = 2/100 #cm
d1max = 7/100 #cm
d2 = 2/100 #cm
d3 = 4.3/100 #cm
cube_w = 5/100 #cm

#Tseinit = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]
#Tscinit = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
#Tscfinal = [[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]]
#Tcestandoff = [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, d2+cube_w], [0, 0, 0, 1]]
theta_y = 3*pi/4
Tcestandoff = [[cos(theta_y), 0, sin(theta_y), 0], [0, 1, 0, 0], [-sin(theta_y), 0, cos(theta_y), d2+cube_w], [0, 0, 0, 1]]
Tcegrasp = [[cos(theta_y), 0, sin(theta_y), 0], [0, 1, 0, 0], [-sin(theta_y), 0, cos(theta_y), -d2], [0, 0, 0, 1]]

#Tcestandoff = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, d2+cube_w], [0, 0, 0, 1]] #maybe make taller/shorter
#Tcegrasp = [[0, 0, 1, d2], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]] #maybe make taller/shorter
#Tcegrasp = [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, -d2], [0, 0, 0, 1]]
k = 1

Tb0 = np.array([[1, 0, 0, 0.1662], 
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])
Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0],
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T
M0e = np.array([[1, 0, 0, 0.033], 
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])
dt = 0.01

def traj_toSE3(traj):
    SE3 = np.array([[traj[0], traj[1], traj[2], traj[9]],
                    [traj[3], traj[4], traj[5], traj[10]],
                    [traj[6], traj[7], traj[8], traj[11]],
                    [0, 0, 0, 1]])
    return SE3

def GenPathwControl(Tscinit, Tscfinal, Tsbot_a_i, Tseinit, K):
    ref_ee_traj = TrajectoryGenerator(Tseinit, Tscinit, Tscfinal, Tcegrasp, Tcestandoff, k)
    a_ee_traj = Tsbot_a_i #initialize actual ee traj
    traj = []
    Xerr_list = []
    Xerr_i = mr.se3ToVec(np.array([[0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]]))
    for ii in range(len(ref_ee_traj)-1):
        Xd = traj_toSE3(ref_ee_traj[ii])
        Xdnext = traj_toSE3(ref_ee_traj[ii+1])
        Tsb = np.array([[cos(a_ee_traj[0]), -sin(a_ee_traj[0]), 0, a_ee_traj[1]],
                        [sin(a_ee_traj[0]), cos(a_ee_traj[0]), 0, a_ee_traj[2]],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])
        thetalist = a_ee_traj[3:8]
        T0e = mr.FKinBody(M0e, Blist, thetalist)
        X = np.matmul(np.matmul(Tsb,Tb0),T0e)
        Ve, Xerr_i, Xerr = FeedbackControl(X, Xd, Xdnext, K[0], K[1], dt, Xerr_i)
        Je = getJe(thetalist) 
        speeds = getSpeeds(Je, Ve)
        controls_vector = [speeds[4], speeds[5], speeds[6], speeds[7], speeds[8], speeds[0], speeds[1], speeds[2], speeds[3]]
        later_config = NextState(a_ee_traj[0:12], controls_vector, dt, 10)
        
        violations = testJointLimits(later_config[3:8])
        while sum(violations) > 0:
            for jj in range(len(violations)):
                if violations[jj] == 1:
                    #print("violated: ", jj)
                    Je[:,jj] = 0
            speeds = getSpeeds(Je, Ve)
            controls_vector = [speeds[4], speeds[5], speeds[6], speeds[7], speeds[8], speeds[0], speeds[1], speeds[2], speeds[3]]
            later_config = NextState(a_ee_traj[0:12], controls_vector, dt, 10)
            violations = testJointLimits(later_config)
        
        later_config.append(ref_ee_traj[ii][-1]) # add gripper state from ref traj
        traj.append(later_config)
        Xerr_list.append(Xerr) #assume k=1
        a_ee_traj = later_config
    return traj, Xerr_list
