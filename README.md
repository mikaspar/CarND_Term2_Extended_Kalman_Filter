# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This project includes source files included in folder src:

main.cpp - establishes the connection with simulator, reads out input data and writes the vehicle position estimateted by EKF.
FusionEKF.cpp - initializes and desribes the system states and covariances. Calls the prediction and update steps. 
FusionEKF.h - declares the FusionEKF class
kalman_filter.cpp - calculates the Predict, Update(Lidar) and UpdateEKF(Radar) functions
kalman_filter.h - declares KalmanFilter class and its methods
tools.cpp - calculates RMSE and Jacobian matrix for radar measurement
tools.h - declares tools class and its methods

## Result
I obtained following resultw when using both Radar and Lidar data for the position and velocity estimation:


Using a measurement of one sensor only leads to lower accuracy of the estimate.

Radar only:

Lidar only:



