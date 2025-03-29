#include "kalmanFilter.hpp"

KalmanFilter::KalmanFilter(int num_states, int num_sensors, 
                           const Eigen::MatrixXd& motion_model, 
                           const Eigen::MatrixXd& measurement_model) 
    : state_dim(num_states), sensor_dim(num_sensors),
      A(motion_model), H(measurement_model), time_(0) {

    // Identity matrix
    I = Eigen::MatrixXd::Identity(state_dim, state_dim);

    // Initialize state vector and covariance
    x = Eigen::VectorXd::Zero(state_dim);
    P = Eigen::MatrixXd::Identity(state_dim, state_dim);

    // Default process noise and measurement noise
    Q = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.01;
    R = Eigen::MatrixXd::Identity(sensor_dim, sensor_dim) * 0.1;

    // Create a mask matrix for replacing T and T²/2
    A_mask = Eigen::MatrixXd::Zero(state_dim, state_dim);
    for (int i = 0; i < state_dim; ++i) {
        for (int j = 0; j < state_dim; ++j) {
            if (A(i, j) == -1)      A_mask(i, j) = 1;  // Mark T
            else if (A(i, j) == -2) A_mask(i, j) = 2;  // Mark T²/2
        }
    }
}

void KalmanFilter::predict(float currentTime) {
    double dt = static_cast<double>(currentTime - time_);
    time_ = currentTime;

    // Modify motion model with time delta
    Eigen::MatrixXd A_dt = A;
    for (int i = 0; i < state_dim; ++i) {
        for (int j = 0; j < state_dim; ++j) {
            if (A_mask(i, j) == 1)      A_dt(i, j) = dt;        // Replace T with dt
            else if (A_mask(i, j) == 2) A_dt(i, j) = (dt * dt) / 2.0; // Replace T²/2 with dt²/2
        }
    }

    // Predict step
    x = A_dt * x;
    P = A_dt * P * A_dt.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    Eigen::VectorXd y = z - H * x;                 // Innovation
    Eigen::MatrixXd S = H * P * H.transpose() + R; // Innovation covariance
    Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain

    // Update state
    x = x + K * y;
    P = (I - K * H) * P;
}
