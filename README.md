# youBot_Control_System

The youBot Control System project was done for Northwestern University’s [Robotics Manipulation (MECH_ENG 449)](https://www.mccormick.northwestern.edu/mechanical/academics/courses/descriptions/449-robotic-manipulation.html) course’s capstone assignment, and it entails generating the trajectory for a mobile 5R robot with four [mecanum wheels](https://en.wikipedia.org/wiki/Mecanum_wheel) (the [youBot](https://cyberbotics.com/doc/guide/youbot#:~:text=The%20youBot%20is%20a%20mobile,efficiently%20modeled%20using%20asymmetric%20friction.)) to complete the task of picking up a cube at a certain location and dropping it off at another. The robot is able to start at positions outside of the default initial position and quickly correct its actual path back to the reference trajectory using a proportional-integral (PI) feedback controller. The robot was simulated virtually via [CoppeliaSim](https://coppeliarobotics.com/downloads), and the specific scene used to create the simulation can be downloaded [here](http://hades.mech.northwestern.edu/index.php/CoppeliaSim_Introduction#Scene_6:_CSV_Mobile_Manipulation_youBot). Below is an example of the simulation produced:

https://user-images.githubusercontent.com/66648349/147486771-7ff1fc24-7143-489e-8799-87bdbf7a1841.mp4

The project utilizes the following topics, which correspond to different sections of the task:

**Inverse kinematics** was used to generate a reference trajectory of the end-effector (ee) that follows these 8 steps:
1. First travel to the standoff position above the initial position of the cube
2. Drop down to a height where the end-effector can grip the cube
3. Close the gripper
4. Return to the initial standoff position
5. Travel to the standoff position above the final position of the cube
6. Lower the cube down to its final position
7. Open the gripper
8. Return to the standoff position above the final position of the cube

**Odometry** was used to simulate realistic movements for the omniwheels so that they spin according to the way the robot chassis maneuvers the environment.

A **PI feedback controller** was used to model and correct the error dynamics of the robot trajectory, since the actual initial configuration of the end-effector should not be restricted to the initial configuration of the end-effector in the reference trajectory. By utilizing an integral controller on top of a proportional controller, the robot is able to eliminate steady state error while producing relatively stable behavior.

This program generates two CSV files, traj.csv and err.csv, which shows the actual configuration of the robot and its error relative to the reference trajectory respectively. The actual configuration of the robot at each time step is recorded as a vector of 13 values (chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state) where each W represents a wheel angle and the Js represent the angles of the robot arm’s revolute joints. traj.csv is then loaded into Scene 6 in CoppeliaSim to create a visualization of the robot’s behavior. More detail and the actual program can be found in the code folder, and videos as well as the error plots of the simulation can be found in the results folder. Note that the program does not take into account joint angle limits, so at certain points of the simulation the robot arm is able to overlap itself. This can be corrected in future development to generate a more realistic simulation of the youBot.
