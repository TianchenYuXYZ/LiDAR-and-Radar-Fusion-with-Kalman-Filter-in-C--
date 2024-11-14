# Kalman Filter and Jacobian Computation Project

## Overview
This project is a C++ implementation of a **Kalman Filter** system designed for real-time object tracking using sensor data. It includes functionality for both **linear and extended Kalman Filters** to handle data from multiple types of sensors, particularly **radar and laser**. The **Jacobian matrix computation** is implemented to linearize nonlinear measurement equations. Additionally, **Root Mean Square Error (RMSE)** is calculated to evaluate the accuracy of predictions and verify the robustness of the filter in different tracking scenarios.

## Project Structure

- **`main.cpp`**: 
  - The main entry point of the program.
  - Initializes the Kalman Filter and manages the input of measurement data from sensors (laser and radar).
  - Processes data sequentially and uses it to predict and update the tracked state.
  - Outputs tracked states and RMSE values to evaluate the filter's performance.
  
- **`kalman_filter.cpp` & `kalman_filter.h`**:
  - Contains the core Kalman Filter logic.
  - Implements both **Prediction** and **Update** steps, central to the Kalman Filter algorithm.
  - **Prediction**: Uses the previous state and a motion model to predict the object's current state.
  - **Update**: Adjusts predictions based on new measurements, integrating sensor data for optimal accuracy.
  - Differentiates between the linear Kalman Filter (for linear models) and the Extended Kalman Filter (for nonlinear models).
  
- **`tracking.cpp` & `tracking.h`**:
  - Manages integration of both **laser** and **radar** measurements.
  - The tracker class handles switching between measurement types and ensures data is processed correctly, updating the Kalman Filter with each new data point.
  - This class is crucial for handling real-world scenarios where sensor data may vary in type and frequency.

- **`JacobianComputation.cpp`**:
  - Implements the **Jacobian Matrix computation**, which is essential for the Extended Kalman Filter.
  - Since radar measurements are nonlinear, this module linearizes the nonlinear equations by calculating partial derivatives with respect to the state variables.
  - Enables accurate updates to the Kalman Filter by approximating nonlinear transformations.

- **`RMSEPerformanceMetric.cpp`**:
  - Calculates the **Root Mean Square Error (RMSE)**, a common metric for evaluating the accuracy of predicted states against ground truth.
  - The RMSE metric is computed continuously during tracking to provide a quantitative measure of the Kalman Filter’s performance.
  - This module is useful for tuning and improving the filter based on accuracy requirements.

- **`obj_pose-laser-radar-synthetic-input.txt`**:
  - A synthetic dataset containing a mix of laser and radar measurements.
  - **Laser data format**: `[L, px, py, timestamp, ...]`
  - **Radar data format**: `[R, rho, phi, rho_dot, timestamp, ...]`
  - The dataset simulates realistic tracking scenarios by providing timestamped measurements of object positions and velocities in different coordinate systems.

## Dependencies

- **C++11**: Required for modern C++ features.
- **Standard Template Library (STL)**: For handling vectors, matrices, and standard data structures.
- **CMake** (optional): Facilitates building and compiling the project.

## How It Works

1. **Data Processing**:
   - The program loads synthetic radar and laser data from `obj_pose-laser-radar-synthetic-input.txt`.
   - Each measurement is timestamped and alternates between laser and radar data to simulate a real-world sensor environment.

2. **Kalman Filter Operations**:
   - **Prediction Step**: Based on the previous state and a motion model, the Kalman Filter predicts the next position and velocity of the tracked object.
   - **Update Step**: As new measurements arrive, the Kalman Filter uses them to refine the predictions.
     - **Laser Update**: Linear update using Euclidean coordinates (px, py).
     - **Radar Update**: Nonlinear update using polar coordinates (rho, phi, rho_dot). This requires Jacobian computation to approximate the nonlinear transformation.

3. **RMSE Evaluation**:
   - RMSE values are calculated throughout the program to assess the accuracy of the predictions.
   - Lower RMSE values indicate better tracking performance, providing a clear metric for tuning the filter.

## Example Usage

1. **Compilation**:
   Compile the project using `g++` or with `CMake`:
   ```bash
   g++ main.cpp kalman_filter.cpp tracking.cpp JacobianComputation.cpp RMSEPerformanceMetric.cpp -o kalman_filter_project
2. **Execution**:
   Run the executable with the synthetic data file as input:
   ./kalman_filter_project obj_pose-laser-radar-synthetic-input.txt
3. **Sample Output**
   The program outputs each predicted state alongside the actual measurements.
   RMSE values are also output periodically to show the tracking performance over time.

## Detailed Explanation of Key Components

### Kalman Filter

The Kalman Filter is an algorithm that estimates the state of a dynamic system by predicting its future state based on previous observations and correcting the prediction using new measurements. This project implements:
- **Prediction** and **Update** steps for both linear and nonlinear measurement models.
- **Noise covariance matrices** tailored to sensor noise characteristics, enhancing robustness.

### Extended Kalman Filter (EKF)

The EKF is used for **nonlinear measurements** (e.g., radar) by linearizing around the current estimate via Jacobian matrices. This allows radar measurements in polar coordinates to be converted into Cartesian coordinates, making them compatible with the Kalman Filter.

### Jacobian Matrix Computation

Since radar measurements involve nonlinear equations, we compute the Jacobian matrix to approximate these equations linearly. This matrix is recalculated with each radar measurement to ensure accuracy in the Kalman Filter’s update step.

### RMSE Performance Metric

RMSE is calculated between the estimated state and the actual measurements, providing a quantitative measure of tracking performance. It’s a key metric for tuning and validating the filter, ensuring it meets accuracy requirements for various applications.

## Application Scenarios

This Kalman Filter setup is applicable in scenarios where real-time tracking of objects is necessary, including:
- **Autonomous Vehicles**: For tracking nearby vehicles or pedestrians using radar and lidar.
- **Robotics**: For positioning and navigating in dynamic environments.
- **Aerospace**: For tracking flying objects or spacecraft with radar.


