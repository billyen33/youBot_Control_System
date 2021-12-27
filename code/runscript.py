import numpy as np
from numpy import pi
import ControlledPath

# Initialize variables
Tseinit = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]
Tscinit = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
Tscfinal = [[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]]

# Set gains Kp and Ki
Kp = 2*np.identity(6)
Ki = 0.1*np.identity(6)
K = [Kp, Ki]

#change this to edit initial configuration of the robot:
# in format [phi, x, y, theta1, theta2, theta3, theta4, theta5, w1, w2, w3, w4, gripperstate]
Tsbot_a_i = [pi/6, 0.2, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0]

print("Generating trajectory...")
traj, Xerr = ControlledPath.GenPathwControl(Tscinit, Tscfinal, Tsbot_a_i, Tseinit, K)

#save traj vectors in a CSV with each vector as its own row
import csv
print("Generating animation CSV file")
with open("D:/Northwestern/ME449/Final_Project/traj.csv", 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(traj)
#save Xerr in a CSV with each vector as its own row
print("Writing error plot data")
with open("D:/Northwestern/ME449/Final_Project/Xerr.csv", 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(Xerr)

print("Done!")