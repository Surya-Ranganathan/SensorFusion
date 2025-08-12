#include "LKF.hpp"

KalmanFilter3D::KalmanFilter3D() 
{
    /*
    Description:  
    Constructor for the KalmanFilter3D class. Initializes the state vector, covariance matrix, 
    state transition matrix, measurement matrix, process noise covariance, and measurement noise covariance.
    
    Input:  
    None.
    
    Output:  
    Initializes internal matrices and state vector for the Kalman filter.
    */

    // State vector [x, vx, ax, y, vy, ay, z, vz, az]
    x = Eigen::VectorXd::Zero(9);

    // Initial covariance
    P = Eigen::MatrixXd::Identity(9, 9) * 500;

    // State transition matrix
    F = Eigen::MatrixXd::Identity(9, 9);
    for (int i = 0; i < 3; ++i) 
    {
        int j = i * 3;

        F(j, j + 1) = dt;
        F(j, j + 2) = 0.5 * dt * dt;
        F(j + 1, j + 2) = dt;
    }

    // Measurement matrix (we observe x, y, z)
    H = Eigen::MatrixXd::Zero(3, 9);
    H(0, 0) = 1;
    H(1, 3) = 1;
    H(2, 6) = 1;

    // Measurement noise
    R = Eigen::MatrixXd::Identity(3, 3) * meas_std * meas_std;

    // Process noise
    double q = accel_std * accel_std;
    Q = Eigen::MatrixXd::Zero(9, 9);

    for (int i = 0; i < 3; ++i) 
    {
        int j = i * 3;
        Q(j, j)     = pow(dt, 4) / 4;
        Q(j, j + 1) = pow(dt, 3) / 2;
        Q(j, j + 2) = pow(dt, 2) / 2;

        Q(j + 1, j)     = pow(dt, 3) / 2;
        Q(j + 1, j + 1) = pow(dt, 2);
        Q(j + 1, j + 2) = dt;

        Q(j + 2, j)     = pow(dt, 2) / 2;
        Q(j + 2, j + 1) = dt;
        Q(j + 2, j + 2) = 1;
    }

    Q *= q;
}

void KalmanFilter3D::predict() 
{
    /*
    Description:  
    Performs the prediction step of the Kalman Filter. Computes the next state and updates the state 
    covariance based on the state transition matrix and process noise.
    
    Input:  
    None (uses internal class state vector x, covariance matrix P, and other matrices).
    
    Output:  
    Updates the internal state vector x and covariance matrix P.
    */

    x = F * x;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter3D::update(const Eigen::Vector3d& z) 
{
    /*
    Description:  
    Performs the update step of the Kalman Filter. Adjusts the predicted state using the incoming measurement z 
    and refines the state and covariance estimates.
    
    Input:  
    z (Eigen::Vector3d): A 3D measurement vector (x, y, z).
    
    Output:  
    Updates the internal state vector x and covariance matrix P based on the Kalman Gain and measurement residual.
    */

    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    x = x + K * (z - H * x);
    P = (Eigen::MatrixXd::Identity(9, 9) - K * H) * P;
}

void KalmanFilter3D::processMeasurements(const std::string& filename) 
{
    /*
    Description:  
    Reads the measurement data from a CSV file, processes each measurement using the Kalman filter, 
    and writes the filtered state estimates to an output file. It iteratively calls predict() and update() for each measurement.
    
    Input:  
    filename (std::string): The path to the CSV file containing 3D measurement data (x, y, z).
    
    Output:  
    Writes the filtered state (position, velocity, and acceleration) to a CSV file kf_output.csv. 
    The output contains the estimated states (x, y, z, vx, vy, vz, ax, ay, az) at each time step.
    */
   
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line);

    std::ofstream output("output/kf_output.csv");
    output << "x,y,z,vx,vy,vz,ax,ay,az\n";

    while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        std::string val;
        std::vector<double> values;

        while (std::getline(ss, val, ','))
            values.push_back(std::stod(val));

        if (values.size() < 3) continue;

        Eigen::Vector3d z;
        z << values[0], values[1], values[2];

        predict();
        update(z);

        output << x(0) << "," << x(3) << "," << x(6) << "," 
                << x(1) << "," << x(4) << "," << x(7) << "," 
                << x(2) << "," << x(5) << "," << x(8) << "\n";
    }

    output.close();
}