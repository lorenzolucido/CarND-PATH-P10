# **Car Control Project: Path Planning**
#### _Lorenzo's version_
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[results]: ./behavior_planning.png

![alt text][results]
---

## Project Introduction
In this project our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Car localization and sensor fusion data are provided, as well as a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Implementation
The core of the project sits on two main functions, both in `behavioral_planner_functions.h`:
- `behavior_planner_simple()`: decides which lane and speed to target
- `generate_trajectory_spline()`: for a particular target speed and lane, generate a smooth trajectory

#### 1. Behavior Planning
The function implements a very simple finite state machine with 3 states:
  1. _if no car in front_ => Stay in your lane and gradually match target speed (49.5 mph)
  2. _if another car is close_ => Switch to lane with maximum available space in front
  3. _if 2. not possible_ => Stay in your lane and decelerate until front car is at safe distance or lane becomes available

#### 2. Trajectory Generation
Here I used this [spline](http://kluge.in-chemnitz.de/opensource/spline/) small library in order to compute a smooth path. Steps are the following:
1. Take 5 evenly-spaced reference points for the trajectory: 2 points on the current or past position of the car and 3 points on the target lane
2. Generate the spline using the piecewise-linear trajectory defined by the 5 points in 1.

## Parameter tuning
The only parameter in the project is `SAFETY_DISTANCE` (in `behavioral_planner_functions.h`). This parameters controls both the distance the self-driving car will assume dangerous when following a vehicle, as well as the space required to do a lane change: +/- 1.5 * SAFETY_DISTANCE.
I have set this parameter to 20 meters, which means if another car in 20 meters ahead in the same lane, than our car will try to switch lane only if there is no car in between -30 meters and +30 meters.

### Simulator.
Simulator required to interact with the path planner is located [here](https://github.com/udacity/self-driving-car-sim/releases).

## Basic Build Instructions

1. Clone this repo.
2. Build it: `./build.sh`.
3. Run it: `cd build && ./path_planning`.


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
