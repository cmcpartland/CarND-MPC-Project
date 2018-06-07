# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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


## The Model
A standard bicycle model is used to model the vehicle motion. The bicycle model utilizes the
vehicle position, velocity, and orientation and incorporates actuators that can introduce an
orientation change ( and a velocity change ( a ) in the following motion δ) update equations:

xt+1 = xt + vt cos(ψt) dt
yt+1 = yt + vt sin(ψt) dt
ψt+1 = ψt + (vt/Lf) δ dt
vt+1 = vt + at dt

where L f is the distance from the center of gravity to the front-end of the vehicle.
The cross-track error ( cte ) and the orientation error ( epsi ) are based on the desired state of the
vehicle in the immediate future. They are updated with the following equations:

ctet+1 = f(xt) − yt + vt sin(eψt) dt
eψt+1 = ψt − ψdest + (vt/Lf) δ dt

where the function f(x) is the polynomial fit to the waypoints along the center of the track.
The state of the vehicle is then defined to be a 6-dimensional vector:
[x, y, ψ, v, cte, eψ]

## Timestep Length and Elapsed Duration
The vehicle state and the target trajectory must both be discretized by the chosen timestep
length, and the amount of points is defined by the chosen elapsed duration. For this project, a
timestep length of dt = 0.1 seconds was chosen and an elapsed duration of 1 second was used.
This results in N = 10 discrete steps used to predict the vehicle state as the model looks 1
second into the future.

At first, the values used were N = 25 and dt = 0.05 since these were used in the lessons.
Because of other issues, I used the video walkthrough provided by the course and tried to use
the instructors used, which are the ones above. These parameters seemed to be successful.
Next, I wanted to see if decreasing dt from 0.1 to 0.05 would increase performance, since this
would mean the controller would be more granular. However, this resulted in horrible
performance. My conclusion was that because the value of dt was less than the latency, the
latency compensation was invalid. Therefore, I bumped dt back to 0.1. I also tried decreasing
computation cost by increasing dt to 0.2, but this resulted in a more conservative controller that
drove very slowly and really hugged the inside of corners.


## Polynomial Fitting and MPC Preprocessing
In order to simplify the calculation of the cross-track error, the waypoints are transformed from
the global reference frame to the reference frame of the vehicle. A 3rd degree polynomial is
then fit to the transformed way points.

In the vehicle frame, the vehicle is located at (0, 0) and has an orientation of 0 radians. The cte
is then simply equal to the value of polynomial at x = 0, and the epsi value is simply equal to the
negative of the slope of the tangent to the polynomial at x = 0 .


## Model Predictive Control with Latency
The latency between decision and actuation was handled by taking the current vehicle state,
simulating the state change over the duration of the latency, and then using this updated state
as the initial state.

The bicycle motion model was used to move the vehicle into the future considering the current
throttle value and the current steering angle. The vehicle state in the global frame was being
considered here, so while the same update equations are valid, there are a couple small
changes.

xt+latency = xt + vt cos(ψt,global) Δ
yt+latency = yt + vt sin(ψt,global) Δ
ψt+latency = ψt − Lf
δt Δ
vt+latency = vt + at Δ

where Δ = latency duration, a = throttle value, δ = steering value, and ψt,global = -
ψt δt

is the updated vehicle orientation in the vehicle reference frame (keeping in mind that the
simulator uses a left-handed coordinate system, so that we must subtract the steering value
from the orientation angle).

The limitation of this latency compensation is that if dt is shorter than the latency duration, then
an actuation is applied before the latency duration ends and the simulation is expired and
invalid. I suppose in some sense, it wouldn’t make sense to try and figure out what actuator
input is needed faster than the actuation can actually be applied, but it should be kept in mind
and documented.

