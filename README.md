# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./figs/example.gif "simulator result"

---
### Project summarize
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data are provided, there is also a sparse map list of waypoints around the highway.

With the implemented codes, the car can go as close as possible to the 50 MPH speed limit, and pass slower traffic when possible. The car could avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

Also the car can travel 28 miles with total acceleration under 10 m/s^2 and jerk under than 10 m/s^3 without incident. Below is an exemple of the code running in the simulator.

![alt text][image1]

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The simulator outputs a previous path that is not realized by the simulator, in the code we return a path that extends this previous path to make sure it has a smooth transition with the last path.

2. Frenet is used thoughout the project to facilitate control and calculation, C++ Spline tool is applied to generate a smooth drivable path, path generation and call for FSM (Finite State Machine) are implemented in `main`.

3. Besides of `main`, several functions are created for FSM: `get_lane`, `lane_speed`, `get_vehicle_ahead`, `get_vehicle_behind`, `successor_states`, `realize_next_state`, `calculate_cost` and `choose_next_state`.

4. `get_lane`: get current lane number (0,1,2) with simulator data.

5. `lane_speed`: get lane speed with a given lane number and sensor fusion data.

6. `get_vehicle_ahead` and `get_vehicle_behind`: search for vehicle ahead/behind of the ego car with a given lane number and a defined search range.

7. `successor_states`: three states are available in the FSM, Keep Lane (KL), Lane Change Left (LCL) and Lane Change Right (LCR), this function determine next possible states based on current state.

8. `realize_next_state`: once given a state action, realize it when safe.

9. `calculate_cost`: calculate cost for a possible next state, the cost is calculated with the speed of intended lane and s distance from nearby vehicles on such lane.

10. `choose_next_state`: FSM call function, determine the best next state with calculated cost.

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



