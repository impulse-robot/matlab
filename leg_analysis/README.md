## Leg analysis
The following scripts analyse the leg geometry.
They were tested in  Matlab R2018a.

## leg_kinematics
Run the script leg_kinematics.m which will visualize the leg gemoetry for different input angles and write the forward and differential kinematics for multiple input angles into a csv file as well as the leg geometry.

## jump_height
Run the script jump_height.m which will calculate the theoretical jumping height of the robot. The calculation accounts for the motor model and changing jacobian throughout the jump. 

## joint_forces
Run the script joint_forces.m which will calcualte the forces in each joint w.r.t to the world frame for different input angles, those forces are then written into a csv file.
