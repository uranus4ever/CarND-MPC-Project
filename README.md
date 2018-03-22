# Autonomous Driving with Model Predictive Control


## Objective

This project is to use Model Predictive Control (MPC) to drive a car in a simulator. The server provides reference waypoints via websocket, and we use MPC to compute steering and throttle commands to drive the car. The solution must be robust to 100ms latency, since it might encounter in real-world application.

In this project, the MPC optimize the actuators (steering and throttle), simulate the vehicle trajactory, and minimize the cost like cross-track error.

*Yellow line - polynomial reference way points* 
*Green line - MPC trajectory path*

![GIF][gif]

## System Introduction
### Kinematic model
A kinematic model is implemented to control the vehicle around the track. Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable.

**States:**

x: cars x position
y: cars y position
ψ (psi): vehicle's angle in radians from the x-direction (radians)
ν: vehicle's velocity
cte: cross track error
eψ : orientation error

**Actuator values:**

δ (delta): steering angle
a : acceleration (including throttle and break)

**Update loop:**

![MPC loop][img1]

### Timestep and Elasped Duration (N & dt)

 - N = 10 
 - dt = 0.1 s

The time horizon **(T)** is the duration over which future predictions are made.
```
T = N * dt
```
T should be as large as possible. However, the smaller dt is preferred for finer resolution. And larger N isn't always better due to computational time. Hence it is a trade-off.

### Polynomial Fitting and MPC Preprocessing

The reference way points are given in the map global coordinate, and they are transferred into the car's coordinate. And then a 3rd order polynomial is fitted to way points.

### Model Predictive Control with Latency

In reality, no actuctor could execute command instaly - there will be a delay as the command propagates through the system. A realistic delay might be 100 milliseconds, as set into the project. The following codes deal with the latency issue.

```
const double latency = 0.1;  // 100 ms
const double px_act = v * latency;
const double py_act = 0;
const double psi_act = - v * steering_angle * latency / Lf;
const double v_act = v + throttle * latency;
const double cte_act = cte + v * sin(epsi) * latency;
const double epsi_act = epsi + psi_act;
// State in vehicle coordinate.
Eigen::VectorXd state(6);
state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
```

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

[//]: # (Image References)
[img1]: ./extra/MPC_loop.PNG
[gif]: ./extra/gif_1.gif