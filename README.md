# Run Away Robot with Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

Overview

This repository contains all the code needed to complete the Project 2: Run Away Robot with Unscented Kalman Filter in Udacity's Self-Driving Car Nanodegree.

Project Introduction

In this project, not only do you implement an UKF, but also use it to catch an escaped robot driving in a circular path. 
The run away robot will be being sensed by a stationary sensor, that is able to measure both noisy lidar and radar data. The capture vehicle will need to use these measurements to close in on the run away bot. To capture the run away bot the capture vehicle needs to come within .1 unit distance of its position. However the capture car and the run away bot have the same max velocity, so if the capture vehicle wants to catch the robot, it will need to predict where the robot will be ahead of time.

Running the Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

mkdir build 

cd build 

cmake .. make 

./UnscentedKF

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, ukf.h, and main.cpp which will use some stragety to catch the robot, just going to the robots current esimtated position will not be enough since the capture vehicle is not fast enough. There are a number of different strageties you can use to try to catch the robot, but all will likely involve prediciting where the robot will be in the future which the UKF can do. Also remember that the run away robot is simplying moving a circular path without any noise in its movements.


Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

// current position state of the capture vehicle, called hunter

["hunter_x"]

["hunter_y"]

["hunter_heading"]

// get noisy lidar and radar measurments from the run away robot.

["lidar_measurement"]

["radar_measurement"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["turn"] <= the desired angle of the capture car "hunter" no limit for the anlge

["dist"] <= the desired distance to move the capture car "hunter" can't move faster than run away robot



## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.
