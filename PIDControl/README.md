# PID Controller for autonomous car
In this project, self-driving car drives itself based on control inputs to minimize the Cross Track Error (CTE) for the car using a PID controller. The parameters of the controller are tuned to ensure the car remains on the road and does not jump the lane lines in the simulator. The pipeline is implemented in C++. The controller is tested in a UDACITY provided simulator for a self-driving car. The proportional, derivative, and integral gains were tuned by brute force by observing the controller drive the car in the simulator and making appropriate adjustments (not a desirable tuning method!).

![Alt Text](https://media.giphy.com/media/MsBLKpFjjvQtizgJlS/giphy-downsized.gif)
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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
