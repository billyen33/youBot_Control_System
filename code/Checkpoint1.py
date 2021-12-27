import modern_robotics as mr
from math import tan, cos, sin, acos, pi
import numpy as np
from JointLimit import testJointLimits

def combineList(list_of_lists):
    combined_list = [item for sublist in list_of_lists for item in sublist]
    return combined_list

def NextState(current_config, controls_vector, dt, max_ang_speed):
    # Current_config is in form [chassis phi, chassis x, chassis y,
    #                           arm theta1, arm theta2, arm theta3, arm theta4, arm theta 5
    #                           wheel ang 1, wheel ang 2, wheel ang 3, wheel ang 4]
    # Controls_vector is in form [arm joint speed 1, arm joint speed 2, arm joint speed 1,
    #                             arm joint speed 4, arm joint speed 5, u1, u2, u3, u4]
    # dt is a float indicating timestep
    # max_ang_speed will replace any arm joint speed or u larger than it

    # Break current_config and controls_vector into their components
    chassis_config = current_config[0:3]
    arm_angs = current_config[3:8]
    wheel_angs = current_config[8:12]
    thetadot_raw = controls_vector[0:5] #arm joint speed
    u_raw = controls_vector[5:9] #wheel speed

    # Enforce maximum angular speed limit
    thetadot = []
    for ang in thetadot_raw:
        if abs(ang) > max_ang_speed:
            thetadot.append(max_ang_speed*(ang/abs(ang)))
        else:
            thetadot.append(ang)
    u = []
    for ang in u_raw:
        if abs(ang) > max_ang_speed:
            u.append(max_ang_speed*(ang/abs(ang)))
        else:
            u.append(ang)

    # Generate new arm joint angles based on speed
    new_arm_angs = []
    for ii in range(len(arm_angs)):
        new_arm_angs.append(arm_angs[ii]+thetadot[ii]*dt)
    
    # Generate new arm joint angles based on speed
    new_wheel_angs = []
    for ii in range(len(wheel_angs)):
        #print("how much wheel changed")
        new_wheel_angs.append(wheel_angs[ii]+u[ii]*dt)

    # Use odometry to find new chassis config
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
    Hp = np.linalg.pinv(H)
    wheel_displacement = [dt*x for x in u]

    Vb = np.matmul(Hp, wheel_displacement)
    Vb = mr.VecTose3(np.array([[0], [0], [Vb[0]], [Vb[1]], [Vb[2]], [0]])).astype(float) #make sure we have floats for all element
    Tbb1 = mr.MatrixExp6(Vb)
    Tsb = np.array([[cos(chassis_config[0]), -sin(chassis_config[0]), 0, chassis_config[1]],
                    [sin(chassis_config[0]), cos(chassis_config[0]), 0, chassis_config[2]],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]], dtype=object)

    Tsb1 = np.matmul(Tsb, Tbb1)
    # Pull new_chassis_config_raw from Tsb1
    new_chassis_config = [acos(Tsb1[0][0]), Tsb1[0][-1], Tsb1[1][-1]]
    # Generate final output of 12-vector representing configuration of robot time dt later
    later_config = [new_chassis_config, new_arm_angs, new_wheel_angs]
    later_config = combineList(later_config) #combine subvectors into one vector
    return later_config

dt = 0.01
chassis_config = [0,0,0]
arm_angs = [0,0,0,0,0]
wheel_angs = [0,0,0,0]
thetadot = [0,0,0,0,0] #arm joint speed
u = [-10, 10, -10, 10] #wheel speed

current_config = combineList([chassis_config, arm_angs, wheel_angs])
controls_vector = combineList([thetadot, u])
test = [thetadot, u]
combined_list = [item for sublist in test for item in sublist]
max_ang_speed = 5

traj = []
for ii in range(100): #run for a second
    new = NextState(current_config, controls_vector, dt, max_ang_speed)
    chassis_config = new[0:3]
    arm_angs = new[3:8]
    wheel_angs = new[8:12]
    current_config = combineList([chassis_config, arm_angs, wheel_angs])
    new.append(0) #add gripper state
    traj.append(new) 
#save joint vectors in a CSV with each vector as its own row
import csv
with open("D:/Northwestern/ME449/Final_Project/Yen_Bill_milestone1/checkpoint1.csv", 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(traj)