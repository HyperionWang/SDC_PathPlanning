# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./Doc/DrivingMiles.png
[image2]: ./Doc/video1.mp4
[image3]: ./Doc/video2.mp4
[image4]: ./Doc/DrivingMiles2.png
[image5]: ./Doc/PathPlanningDemo.gif

### Project Specification

The goal of this project is to design and implement a path planner module in the self-driving car to plan the path and to execute the lane change when it is necessary. The requirement for the path planner is to keep the car within its current lane, control the speed without jerk or hitting other cars, and pass the slower moving traffic by using localization, sensor fusion and map data provided by external system. 

 ![Path Planning Demo][image5]

[![SDC Path Planning](http://img.youtube.com/vi/vj810JwMjs0/0.jpg)](http://www.youtube.com/watch?v=vj810JwMjs0 "SDC Path Planning")

### Project rubrics
1. The code compiles correctly.
    No error during the make and the program is running with no error. 
2. The car is able to drive at least 4.43 miles without incident.
    The image below shows the car using the designed path planner is able to drive more than 10 miles (and longer).
    
 ![alt text][image1]

3. Max Acceleration and jerk are not exceeded.
    No accelleration more than 10 m/s^2 or a jerk of 10 m/s^3 happened.
4. Car does not have collisions.
    No collision happened.
5. The car stays in its lane, except for the time between changing lanes.
    The car always stays in its lane, and the lane change path is safe and quick.
6. The car is able to change lanes.
    The car detects the speed of cars around it, and it makes decision to change lane or not based on if the slower car is ahead or not. The detailed explained in the following section.  

### Path Planning Design Description

The path planning has three parts: 
1. read the path from the path data and calculate the waypoint based on car's coordinate
2. Uinsg localization info and map info to find the surrounding cars and their speed, then, to make decision on staying on the same lane or changing the lane.
3. Convert the decision into the final path in near future (90 meters with 30 meter step) and send to the control part
The three steps are repeated in every cycle of waypoint update.

In the step 1, based on the previous waypoint info and car's current status, the car's speed, location, and orientation could be calculated. Those are important and necessary info to make path planning in the second step. The code could be found from line 212 - 275 in main.cpp.

In the step 2, 
(a) the path planner read the surround car's info from sensor_fusion data, and predict the distance between the car and the car in front of it based on current car's speed/position and the front car's speed/position. If the distance is less than the safe distance, then, the car will start to look at the cars at the left and right side, in order to plan the lane change. The front car check code could be found from line 277 to line 315 in main.cpp.

(b)After the front cars check, the car need to adjust the current speed based on the car distance and speed limit, to avoid speed violation and collision.

(c)After the speed check, if the planner decided to change the lane, then, the traffic at the left and right side of the car need to be checked. The planner will prefer to changing to the left lane than the right lane. And the planned would make final decision on changing the lane only if: 1. the car on the target lane is further than the save distance; 2. the car ahead on the target lane has higher speed than the car itself does; 3. the car behind on the target lane has loweer speed than the car does. If the car cannot change the lane, then, the speed of the car would be changed to follow the current lane's car ahead. The code code be found from line 326 to line 386 in main.cpp. 

In the step 3, the final waypoints to drive the car would be calcualated based on the decision of the speed and target lane (changing lane or not). 

### Results
The following videos show the car is running on the road with good lane change and path planning. 
The videos are stored in Doc folder.
![Driving 13 Miles][image4]
![Video1][image2]
![video2][image3]

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

