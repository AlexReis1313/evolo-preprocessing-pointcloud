#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

class KalmanFilter {
public:
    int state_dim, sensor_dim;
    rclcpp::Time  time_;
    double acell_cov_R_, pose_cov_Q_, boundingBox_cov_Q_, covariance_limit_factor_;
    Eigen::MatrixXd A, A_mask, H, Q, R, P, I;
    Eigen::VectorXd x, x_boundingBox;

    KalmanFilter(double x_init, double y_init, int num_states, int num_sensors, 
                const Eigen::MatrixXd& motion_model, 
                const Eigen::MatrixXd& measurement_model,
                const Eigen::MatrixXd& process_noise,
                double a_cov, double xy_cov, double bb_cov, double cov_limit_factor);

    void predict(rclcpp::Time currentTime);
    void update(const Eigen::VectorXd& z);
    Eigen::VectorXd predictTheFuture(float deltaT);
    std::pair<double &, double &> produceBoundingBox_withCov(bool verbose);
};

#endif // KALMANFILTER_H
