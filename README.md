
# CarND-MPC-Project

## Self-Driving Car Engineer Nanodegree Program

## Project 5: MPC

---

The goal of this project is to implement a MPC to set steering angle and throttle for a vehicle.

As usual the model itself is fairly straightforward. The difficulty - otherwise known as the interesting part - lies in setting hyperparameters and in tuning the model via multipliers for components of the cost function. Details below.

### Usage

1. Run the executable resulting from building the code in this repository.
1. Run the Udacity term2 simulator. Select the "Project 5: MPC Controller" option.

No further action is required from the user. If all goes well the vehicle will drive around the track and remain on the road at all times.

### MPC

In this project we use a Model Predictive Controller (MPC) to set both the steering angle and throttle value for a vehicle based on minimizing a cost function. This is an example of a kinematic vehicle model, relatively simple and easy to implement which means that it can be updated in real time.

MPC implementation consists of a cost function and a series of constraints. The solver will derive values for the actuators (steering and throttle) based on minimizing the cost function withint the specified constraints.

#### Cost Function

The cost function can include many terms. For this project I added cost terms for cross track error (CTE) and orientation error (EPSI). I weighted each of these terms very highly as these are the portions of the cost function designed to keep the car on the road.
I also added a term for the difference between the current car velocity and a reference velocity. This term is designed to prevent the car from stopping in case all other cost terms are minimized.

I then added a couple of terms to the cost function designed to minimize steering and throttle changes. Finally I added a couple of terms designed to minimize the rate of steering and throttle actuator changes. These final four terms do nothing for keeping the car on the road but are necessary to smooth the ride, and important, albeit secondary, goal.

#### Constraints

In addition to the cost function the model implementation consists of adding constraints. At a high level simple constraints were added to keep steering changes in the range [-25 degrees, +25 degrees] and to keep throttle changes in the range [-1.0, +1.0]. The goal of these constraints is to prevent large changes in steering - which may lead to the car becoming unstable and leaving the road - and in acceleration - which really pertains more to a smoother ride than anything else.

The bulk of the constraints though are just the model state transition equations. The model state consists of six values: [x, y, psi, v, cte, epsi]. The first four represent the car's current position, orientation and speed. The last two represent the current cross track error and orientation error (in both cases this is just the difference between the desired value and the current value).

[MPC model equations](doc/MPC.pdf)

### Setting Hyperparameters

For this project there are really only two hyperparameters of note - total number of timesteps (N) and the time increment (dt) used to run successive steps of the model. The product of these two values (N * dt) determines the number of model steps for each telemetry packet received from the simulator.

I began with the values used for the previous lessons, i.e., N=25 and dt=0.05. Via nothing more than trial and error I settled on values of N=10 and dt=0.1. The reasoning behind reducing the value of N was the sharp curves in the simulated road track. A larger value with a smaller delta time value led to the vehicle being unable to respond to the curves quickly enough. The reason for increasing the value of dt from 0.05 to 0.1 was simply to improve the performance of the model. Cutting the number of calculations in half greating improved performance while not sacrificing much accuracy.

### The Code

Following the directory structure in the original CarND-MPC-Project repository, all C++ source code for the PID controller project is in the 'src' subdirectory.

MPC constraints and cost terms are defined in MPC.[h|cpp]. The layout is essentially the same as that used for the in-class lessons. The only real difference is the values used for N and dt as described above and the tuning multpliers for the various cost terms, also as described above.

In main.cpp we have the code for communicating with the simulator. For each telemetry packet received from the simulator ([simulator data](doc/DATA.md)) the operation of the model is quite simple.

1. Translate the given waypoints from global coordinates to car coordinates.
1. Fit a third degree polynomial to these waypoints. This leads to a set of four coefficients for the third degree polynomial. We also generate the three coefficients for the polynomial derivative at this point.
1. Use the polynomial and derivative coefficients to calculate the current cross track error and orientation error.
1. Set the initial model state to 0 position and orientation since we've already translated the waypoints to car coordinates. We use the velocity from the telemetry packet and the calculated cte and epsi values.
1. Run the solver using the initial state and the waypoint polynomial coefficients.
1. Update the vehicle steering angle and throttle value. Two slight wrinkles with setting the steering angle - our model equations assume that a positive steering angle indicates a left turn while a positive steering angle in the simulator indicates a right turn. This is easly resolve by multiplying the value calculated by the solver by -1. The other issue is that we set a constraint on the solver to keep the steering angle in the range [-25 degrees, -25 degrees] while the simulator requires a steering angle in the range [-1, +1]. This is also easily resolve by dividing the solver-calculated steering angle by 25 degrees (converted to radians of course).
1. This is the end of the model step but for visualizing results we also plot the predicted points generated by the solver as well as the points along the polynomial generated from the original waypoints.

[Project Instructions (original README file)](doc/project.md)

### Conclusion

This was another interesting project. Similar to the PID controller project (assuming a second PID controller was used to control throttle) and yet quite different in its operation. Both models seem to have strengths and weaknesses and I can see that either might provide better solutions in different situations. Perhaps a combination of both? Seems impractical though. One advantage of the MPC is that one model is used to generate both a steering angle and a throttle value wheras we require two PID controlls - one for each actuator.

Unfortunately I ended up spending at least as much time setting up my environment for this project as I did working on the project. It proved to be quite impossible to build run the project at all in Windows and getting a suitable Linux-like environment set up with all approproiate libraries installed without actual access to a Linux machine was quite time consuming.
