In this new task, new initial and goal configurations of the cube are introduced
to test the feedforward-plus-P controller obtained from the best case, where the
Kp gain is set as 1.3. The new initial and goal configurations of the cube are 
listed in the following. Overall, the convergence of the Xerr appears before the
end of the first moving stage and is sustained until the end of the simulation.
The 5R arm robot is able to transfer the cube from the new initial position to the
new desired position using the feedforward-plus-p controller obtained from the best
case.

The initial configuration of the cube w.r.t the fixed frame {s} is
Tsc_initial = [[1,0,0,1.2]
               [0,1,0,0.5]
               [0,0,1,0.025]
               [0,0,0,1]]
the goal configuration of the cube w.r.t the fixed frame {s} is
Tsc_goal = [[0,1,0,0.5]
            [-1,0,0,-1.2]
            [0,0,1,0.025]
            [0,0,0,1]]
