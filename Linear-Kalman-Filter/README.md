# Linear-Kalman-Filter

This repository contains a C++ implementation of a Linear Kalman Filter (LKF) for estimating position, velocity, and acceleration from simulated 3D measurement data.

The **Kalman filter** is an algorithm used to estimate the true state of a system that evolves over time, especially when measurements are noisy or imperfect. It is widely used in fields like robotics, navigation, tracking, and sensor fusion due to its efficiency and accuracy.

## How it Works

At each time step, the filter performs two main operations:

1. **Prediction**  
   - Predicts the next state of the system based on its current state and how it is expected to change.

2. **Update**  
   - Incorporates new measurement data to correct and refine the prediction, considering both the model's and the measurement's uncertainties.

## Key Equations

The **Kalman filter** runs in two steps: **Prediction** and **Update**.

---

### 1. Prediction Step

$$
\hat{x}_k^- = A \hat{x}_{k-1} + B u_{k-1}
$$

$$
P_k^- = A \, P_{k-1} \, A^T + Q
$$

**Where:**  
- \( \hat{x}_k^- \): Predicted state estimate  
- \( P_k^- \): Predicted covariance estimate  
- \( A \): State transition matrix  
- \( B \): Control input matrix  
- \( u_{k-1} \): Control input at step \( k-1 \)  
- \( Q \): Process noise covariance matrix  

---

### 2. Update Step

**Kalman Gain:**

$$
K_k = P_k^- H^T \left( H \, P_k^- \, H^T + R \right)^{-1}
$$

**State Update:**

$$
\begin{aligned}
\hat{x}_k &= \hat{x}_k^- + K_k \bigl( z_k - H \, \hat{x}_k^- \bigr)
\end{aligned}
$$

**Covariance Update:**

$$
P_k = (I - K_k \, H) \, P_k^-
$$

**Where:** 
 
- \( K_k \): Kalman gain  
- \( z_k \): Measurement at step \( k \)  
- \( H \): Measurement matrix  
- \( R \): Measurement noise covariance  
- \( I \): Identity matrix  

---

### 3. Final Projection Equation (World to Measurement Space)

$$
s \cdot u = K \, [R \, | \, T] \, X
$$

**Where:**  
- \( X \): State vector in world coordinates (homogeneous form)  
- \( R, T \): Rotation & translation (extrinsic parameters)  
- \( K \): Intrinsic matrix  
- \( s \): Scale factor  

---

## Why Use It?

- **Combines predictions and noisy measurements** to get the most accurate estimate possible.  
- Ideal for **tracking, robotics, navigation, and sensor fusion** applications.  
- **Lightweight and efficient** — suitable for real-time systems.

---

**Example Applications:**  
- GPS and IMU sensor fusion  
- Object and position tracking  
- Robot or vehicle localization

---

## Setup Instructions

### Step 1: Install Required Dependencies

```bash
sudo apt update && sudo apt upgrade
sudo apt install build-essential cmake

# Install Eigen
mkdir eigen && cd eigen
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xzvf eigen-3.4.0.tar.gz
cd eigen-3.4.0
mkdir build && cd build
sudo cmake ..
sudo make install
sudo cp -r /usr/local/include/eigen3/Eigen /usr/local/include/

# Install Python 3 and required packages
sudo apt install python3 python3-pip
pip3 install numpy matplotlib pandas

```

### Step 2: Build, Run, Visualize, and Clean
```bash

# Run the full pipeline including build and plot
bash run.sh 

# Clean build artifacts
make clean
```

## Output

After running the simulation and the plotting script, you will see:

- Figure 1: Trajectory vs Stanley Controller Path
- Figure 2: Velocity vs Time (Actual vs Target)
- Figure 3: Theta (Yaw Angle) vs Time

## Directory Structure

```
.
├── data/
│   └── target_tracking.csv       # Input: 3D position data (x, y, z)
├── include/
│   ├── LKF.hpp                   # Kalman Filter header
│   └── main.hpp                  # Main program header
├── script/
│   └── plot.py                   # Python script for plotting output
├── src/
│   ├── LKF.cpp                   # Kalman Filter implementation
│   └── main.cpp                  # Main program implementation
├── makefile                      # Build instructions
├── run.sh                        # Script to compile and run everything
└── README.md                     # Project documentation
```