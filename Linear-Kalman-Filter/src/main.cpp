#include "main.hpp"

int main() 
{
    /*
    Description:
    This C++ program uses a 3D Kalman Filter to estimate position, velocity, 
    and acceleration of an object based on noisy 3D position measurements.

    Input:
    - 3D position measurements (x, y, z) from the CSV file `target_tracking.csv`.

    Output:
    - Filtered position, velocity, and acceleration written to `kf_output.csv`.
    */
   
    KalmanFilter3D kf;
    kf.processMeasurements("data/target_tracking.csv");
    
    std::cout << "3D Kalman Filter complete. Output written to kf_output.csv\n";
    return 0;
}