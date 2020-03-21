# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

This project utilizes an Extended Kalman Filter to estimate the state of a moving object with noisy lidar and radar measurements. By passing the radar and ladar readings through the Extended Kalman Filter, an output with a higher degree of certainty and also a lower RMSE value when compared to ground truth values was gotten.

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

After passing the sensor data form './data/obj_pose-laser-radar-synthetic-input.txt' through the Extended Kalman Filter, the result can be simulated by running the './src/display.py' file.
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

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.