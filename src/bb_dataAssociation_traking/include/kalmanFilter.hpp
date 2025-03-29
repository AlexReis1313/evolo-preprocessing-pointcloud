#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    int state_dim, sensor_dim;
    float time_;
    Eigen::MatrixXd A, A_mask, H, Q, R, P, I;
    Eigen::VectorXd x;

    KalmanFilter(int num_states, int num_sensors, 
                 const Eigen::MatrixXd& motion_model, 
                 const Eigen::MatrixXd& measurement_model);

    void predict(float currentTime);
    void update(const Eigen::VectorXd& z);
};

#endif // KALMANFILTER_H
