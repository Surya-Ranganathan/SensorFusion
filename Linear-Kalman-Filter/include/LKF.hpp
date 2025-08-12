/*
Author      :   Surya R
Date        :   10 April 2025
Description :   This header file contains the class and functions for a 3D Kalman Filter (LKF) implementation.
                It includes methods for predicting the next state, updating the filter with new measurements,
                and processing measurements from a file.
*/

#ifndef LKF_HPP
#define LKF_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

#define dt           1.0
#define accel_std    0.1
#define meas_std     1.0

class KalmanFilter3D 
{
    /*
        Description:
        The KalmanFilter3D class is used to estimate position, velocity, and acceleration in 3D space 
        using a Kalman filter. It predicts the state based on a motion model and updates the prediction 
        with new sensor measurements.

        Key functions:
        - Predict: Calculates the next state based on the current state and model.
        - Update: Refines the state estimate using the new measurement.
        - ProcessMeasurements: Reads measurements from a file and runs the filter on them.
    */

private:
    Eigen::VectorXd x;
    Eigen::MatrixXd P, F, Q, H, R;

    void predict();
    void update(const Eigen::Vector3d& z);

public:
    KalmanFilter3D();
    void processMeasurements(const std::string& filename);


};

#endif