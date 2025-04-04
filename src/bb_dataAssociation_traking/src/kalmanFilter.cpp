#include "kalmanFilter.hpp"

KalmanFilter::KalmanFilter(int num_states, int num_sensors, 
                           const Eigen::MatrixXd& motion_model, 
                           const Eigen::MatrixXd& measurement_model,
                           const Eigen::MatrixXd& process_noise,
                            double a_cov, double xy_cov) 
    : state_dim(num_states), sensor_dim(num_sensors),
      A(motion_model), H(measurement_model),Q(process_noise),acell_cov_(a_cov), pose_cov(xy_cov), time_(0)  {

    // Identity matrix
    I = Eigen::MatrixXd::Identity(state_dim, state_dim);

    // Initialize state vector and covariance
    x = Eigen::VectorXd::Zero(state_dim);
    P = Eigen::MatrixXd::Identity(state_dim, state_dim);

    // Default process noise and measurement noise
    //Q = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.5;
    R = Eigen::MatrixXd::Identity(sensor_dim, sensor_dim) * pose_cov;

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
    if (dt>10){
        dt=0.1;
    }
    else if (dt ==0.0){
        std::cout<< "Time dif between cloud messages's stamp is null - no kf tracking is possible. Assuming dt=0.1s to keep running"<<std::endl;
        dt=0.1;
    }
    time_ = currentTime;
    // Modify motion model with time delta
    Eigen::MatrixXd A_dt = A;
    for (int i = 0; i < state_dim; ++i) {
        for (int j = 0; j < state_dim; ++j) {
            if (A_mask(i, j) == 1)      A_dt(i, j) = dt;        // Replace T with dt
            else if (A_mask(i, j) == 2) A_dt(i, j) = (dt * dt) / 2.0; // Replace T²/2 with dt²/2
        }
    }

    //Modify process noise with time delta
    Eigen::MatrixXd Q_dt = Q;
    for (int i = 0; i < state_dim; ++i) {
        for (int j = 0; j < state_dim; ++j) {
            if (Q(i, j) == 4)      Q_dt(i, j) = acell_cov_ * std::pow(dt,4)/4;        // Replace 4 with dt⁴/4
            else if (Q(i, j) == 3) Q_dt(i, j) = acell_cov_ * std::pow(dt,3)/2; // Replace 3 with dt³/2
            else if (Q(i, j) == 2) Q_dt(i, j) = acell_cov_ * std::pow(dt,2); // Replace 2 with dt²
        }
    }

    // Predict step
    x = A_dt * x;
    P = A_dt * P * A_dt.transpose() + Q_dt;
}



void KalmanFilter::update(const Eigen::VectorXd& z) {
    std::cout << "updating  ";

    Eigen::VectorXd y = z - H * x;                 // Innovation
    Eigen::MatrixXd S = H * P * H.transpose() + R; // Innovation covariance
    Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain
    std::cout << "y:"<<y<< "  , K: "<< K << std::endl;
    // Update state
    x = x + K * y;
    P = (I - K * H) * P;
    std::cout << "x in update: " << x << std::endl;
}

Eigen::VectorXd KalmanFilter::predictTheFuture(float deltaT) {
    double dt = static_cast<double>(deltaT);

    // Modify motion model with time delta
    Eigen::MatrixXd A_dt = A;
    for (int i = 0; i < state_dim; ++i) {
        for (int j = 0; j < state_dim; ++j) {
            if (A_mask(i, j) == 1)      A_dt(i, j) = dt;        // Replace T with dt
            else if (A_mask(i, j) == 2) A_dt(i, j) = (dt * dt) / 2.0; // Replace T²/2 with dt²/2
        }
    }

    // Predict step
    return A_dt * x;
}