# Sensor Fusion

This repository contains a C++ implementation of sensor fusion, estimating position, velocity, and acceleration from simulated 3D measurement data.

Currently, the implementation uses **LiDAR** measurement data for state estimation.

## Planned Extensions

Future versions will support additional sensor fusion algorithms, including:

- **Kalman Filter (KF)** / **Extended Kalman Filter (EKF)** / **Unscented Kalman Filter (UKF)** — for both linear and nonlinear estimation
- **Particle Filter** — Bayesian Monte Carlo methods for tracking
- **Complementary Filter** — simple low-pass/high-pass blending
- **Covariance Intersection** — robust fusion with uncertain correlations
- **Deep Learning–Based Fusion** — neural networks for multi-sensor data merging
