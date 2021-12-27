import modern_robotics as mr
from math import tan, cos, sin, pi
import numpy as np

Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0],
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T
M0e = np.array([[1, 0, 0, 0.033], 
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])
Tb0 = np.array([[1, 0, 0, 0.1662], 
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])

def getJe(thetalist):
    # this function generates Je by first creating Jarm and then J of the base
    Ja = mr.JacobianBody(Blist, thetalist)
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    
    # Generate H
    thetalist = [0, 0, 0.2, -1.6, 0]
    l = 0.47 #forward-backward distance between wheels
    w = 0.3 #side-to-side distance between wheels
    wheel_rad = np.array([0.0475, 0.0475, 0.0475, 0.0475]) #define by wiki
    x = np.array([l/2, l/2, -l/2, -l/2]) #x coordinate of wheels in chassis body frame
    y = np.array([w/2, -w/2, -w/2, w/2]) #y corrdinate of wheels in chassis body frame
    gamma = np.array([-pi/4, pi/4, -pi/4, pi/4]) #direction of free sliding
    B = np.array([0,0,0,0]) #driving direction of the four wheels
    drive_vector = []
    lin_v_wheel = []
    lin_v_b = []
    
    H = []
    for ii in range(len(wheel_rad)):
        drive_vector.append((1/wheel_rad[ii])*np.array([1, tan(gamma[ii])]))
        lin_v_wheel.append(np.array([[cos(B[ii]), sin(B[ii])],[-sin(B[ii]), cos(B[ii])]]))
        lin_v_b.append(np.array([[-y[ii], 1, 0], [x[ii], 0, 1]]))
        H.append(np.matmul(np.matmul(drive_vector[-1], lin_v_wheel[-1]), lin_v_b[-1]))
    F = np.linalg.pinv(H)
    F6 = np.array([[0, 0, 0, 0],
                   [0, 0, 0, 0],
                   F[0],
                   F[1],
                   F[2],
                   [0, 0, 0, 0]])
    temp = mr.Adjoint(np.matmul(mr.TransInv(T0e), mr.TransInv(Tb0)))
    Jb = np.matmul(temp, F6)
    Je = np.hstack([Jb, Ja])
    return Je

def getSpeeds(Je, Ve):
    #speeds is in format u1, u2, u3, u4, thetadot1, thetadot2, thetadot3, thetadot4, thetadot5
    speeds = np.matmul(np.linalg.pinv(Je), Ve)
    return(speeds)

def FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt, Xerr_i):
    # Inputs should be in the following form:
    # Tse: current actual ee config
    # Tsed: current ee desired config
    # Tsednext: desired ee config at next timestep in reference traj
    # Kp: proportional gain matrix
    # Ki: integral gain matrix

    Tseinv = mr.TransInv(Tse)
    Tsedinv = mr.TransInv(Tsed)
    Vd = mr.se3ToVec((1/dt)*mr.MatrixLog6(np.matmul(Tsedinv, Tsednext)))
    Xerr = mr.se3ToVec(mr.MatrixLog6(np.matmul(Tseinv, Tsed)))
    Xerr_i = Xerr_i + Xerr*dt
    Ve = np.matmul(mr.Adjoint(np.matmul(Tseinv, Tsed)), Vd) + np.matmul(Kp, Xerr) + np.matmul(Ki, Xerr_i)
    return Ve, Xerr_i, Xerr