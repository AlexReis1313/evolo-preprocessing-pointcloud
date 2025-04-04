#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>


class KalmanFilter {
public:
    int state_dim, sensor_dim;
    double time_=0;
    double acell_cov_, pose_cov;
    Eigen::MatrixXd A, A_mask, H, Q, R, P, I;
    Eigen::VectorXd x;

    KalmanFilter(int num_states, int num_sensors, 
                const Eigen::MatrixXd& motion_model, 
                const Eigen::MatrixXd& measurement_model,
                const Eigen::MatrixXd& process_noise,
                double a_cov, double xy_cov);

    void predict(float currentTime);
    void update(const Eigen::VectorXd& z);
    Eigen::VectorXd predictTheFuture(float deltaT);
};

#endif // KALMANFILTER_H
