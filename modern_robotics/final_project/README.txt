In this project, a software is developed for path planning and feedback control of a youBot 
by following the guidance of the project. With the well-tuned control, the youBot successfully 
transferred a cube from a specified intial location to a desired location. 

In brief, the software consists of three major utility functions (all in utilities.py):
    1. NextState            --- determine next end effector configuration based on current configuration;
    2. TrajectoryGenerator  --- determine a reference trajectory for the end effector moving from a given
                                initial position to a given desired position;
    3. FeedbackControl      --- calculate the end effector twist and then the speeds of wheels and joints
                            by implementing a feedback control to compare actual configuration to the
                            reference trajectory configuration.
Some features are also considered in this software to facilitate the path planning and control of 
the youBot. They are listed as following:
1. The transferring process consists of 8 moving stages, and the chassis base is stationary in the 
moving stage of 2,4,6, and 8;
2. the upper limit of joint speed is 80% of that of chassis wheel speed;
3. limits of joint 2 and 3 are considered in this study.

By integrating the above three functions within a for loop, the resulting actual youBot trajectory,
and errors between actual and reference trajectory configurations can be determined and output as 
.csv files for further analysis. In general, three cases are considered in this project, with the 
feedfrward-plus-p controller implemented: a best case with a well-tuned Kp gain; an overshoot case 
with a less-well-tune Kp gainl; and a new task in which the well-tuned controler is implemented to 
transfer a cube from new initial position to new goal position. For all three cases, the robot can 
successfully transfer the cube from initial position to the desired configuration. 


