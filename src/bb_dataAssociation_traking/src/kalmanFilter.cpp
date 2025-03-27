#include <Eigen/Dense>
#include <iostream>

class KalmanFilter {
public:
    int state_dim, sensor_dim, time;
    Eigen::MatrixXd F, H, Q, R, P, I;
    Eigen::VectorXd x;

    KalmanFilter(int num_states, int num_sensors, 
                 const Eigen::MatrixXd& motion_model, 
                 const Eigen::MatrixXd& measurement_model) 
        : state_dim(num_states), sensor_dim(num_sensors),
          F(motion_model), H(measurement_model) {

        // Identity matrix
        I = Eigen::MatrixXd::Identity(state_dim, state_dim);

        // Initialize state vector and covariance
        x = Eigen::VectorXd::Zero(state_dim);
        P = Eigen::MatrixXd::Identity(state_dim, state_dim);

        // Default process noise and measurement noise
        Q = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.01;
        R = Eigen::MatrixXd::Identity(sensor_dim, sensor_dim) * 0.1;
    }

    void predict(double dt) {
        // Modify motion model with time delta
        Eigen::MatrixXd F_dt = F;
        F_dt(0, 1) = dt;  // Example: Updating for variable time step
        
        // Predict step
        x = F_dt * x;
        P = F_dt * P * F_dt.transpose() + Q;
    }

    void update(const Eigen::VectorXd& z) {
        Eigen::VectorXd y = z - H * x;                 // Innovation
        Eigen::MatrixXd S = H * P * H.transpose() + R; // Innovation covariance
        Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain

        // Update state
        x = x + K * y;
        P = (I - K * H) * P;
    }
};


