# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program


[//]: # (Image References)
[image1]: ./helper_images/Port_Forwarding.png "Port_Forwarding"
[image2]: ./helper_images/Result_Dataset1.png "Result_Dataset1"
[image3]: ./helper_images/Result_Dataset2.png "Result_Dataset2"


In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric, which says: px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1. 

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


## Instructions For Setting Up Environment
This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts. For use in Windows10, below is an instruction for setting up the environment. 

### uWebSocketIO Instructions using docker in Windows10

1. Launch "Docker quickstart terminal"
2. Enter a Docker image that has all the project dependencies by running:

`docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest`

After running this command, we should see a command prompt that looks like:
`root@eb84269b7603:/work#`

3. To exit the docker image, run `exit` after test is done. 

### Port Forwarding Instructions

When using a virtual machine and running the simulator on the host machine, it is critical to set up port forwarding, as described in the following steps.

1. Launch Oracle VM Virtualbox
2. Click on the default session and select settings.
3. Click on Network, and then Advanced.
4. Click on Port Forwarding
5. Click on the green plus, adds new port forwarding rule.
6. Add a rule that assigns 4567 as both the host port and guest Port, as in the follwing image.
![alt text][image1]

### Basic Build Instructions

Perform the following steps in the docker command prompt:

1. Clone this repo.
2. Delete existing build directory: `rm -r build`
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make` 
   * The project instruction says, on windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`. But I did not need to run this.
5. Run it: `./ExtendedKF `
6. Start the simulator `term2_sim.exe` on local machine, select `Project 1/2: EKF and UKF`. If everything is successful, you should see `Connected!!!` in the docker terminal. This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).
7. There are two datasets in the simulator. When changing dataset, the safest way is to re-run `./ExtendedKF ` and re-start the simulator. Sometimes without a proper restart, the RMSE values are cumulated. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2 (three-term version) or Term 1 (two-term version)
of CarND. If you are enrolled, see the Project Resources page in the classroom
for instructions and the project rubric.

## Hints and Tips!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.
* Students have reported rapid expansion of log files when using the term 2 simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.

    + create an empty log file
    + remove write permissions so that the simulator can't write to log
 * Please note that the ```Eigen``` library does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


# Extended Kalman Filter Project Result

## System Model

The system equation is:

x<sub>k+1</sub> = F * x<sub>k</sub> + G * a<sub>k</sub>      <br>

Here a<sub>k</sub> is considered as gaussian with zero mean and covariance matrix 

[&sigma;<sub>x</sub><sup>2</sup>, 0; 0, &sigma;<sub>y</sub><sup>2</sup>].

Then the process noise has covariance 

Q = G * [&sigma;<sub>x</sub><sup>2</sup>, 0; 0, &sigma;<sub>y</sub><sup>2</sup>] * G<sup>T</sup>

For measurement using lidar, we have the measurement matrix

z<sub>k</sub> = H<sub>lidar</sub> * x<sub>k</sub> + w<sub>lidar</sub>, and w<sub>lidar</sub> has covariance R<sub>lidar</sub>

Lidar measures the position in cartesian coordinates.

For measurement using radar, we have the measurement equation

z<sub>k</sub> = h<sub>radar</sub>(x<sub>k</sub>) + w<sub>radar</sub>, and w<sub>lidar</sub> has covariance R<sub>radar</sub>

The jacobian matrix is used for radar when performing extended karman filter. 

H<sub>j,lidar</sub> = &Delta;h(x), and we use the predicted x<sub>k|k</sub> when calculating the jacobian matrix. 

 
## Kalman Filter Equations

For the system equation
x<sub>k+1</sub> = F * x<sub>k</sub> + v      <br>
z<sub>k</sub> = H(x<sub>k</sub>) + w

Initialization:

x<sub>0|-1</sub> = z<sub>0</sub>
P<sub>0|-1</sub> = P<sub>0</sub>

Measurement update step:

K<sub>k</sub> = P<sub>k|k-1</sub>H<sub>k</sub><sup>T</sup>
(H<sub>k</sub>P<sub>k|k-1</sub>H<sub>k</sub><sup>T</sup> + R<sub>k</sub>)<sup>-1</sup>

x<sub>k|k</sub> = x<sub>k|k-1</sub> + K<sub>k</sub> (y<sub>k</sub> - H<sub>k</sub>x<sub>k|k-1</sub>)

P<sub>k|k</sub> = P<sub>k|k-1</sub> - K<sub>k</sub>H<sub>k</sub>P<sub>k|k-1</sub>

Prediction step:

x<sub>k+1|k</sub> = F * x<sub>k|k</sub>
P<sub>k+1|k</sub> = F * P<sub>k|k</sub> * F<sup>T</sup> + Q<sub>k</sub>

## Kalman Filter Results

The estimation result on Dataset 1 & 2 are shown in the following images:

![Result_Dataset1][image2]

![Result_Dataset2][image3]

As shown, the RMSE values for positions and speeds satisfy the project requirement.