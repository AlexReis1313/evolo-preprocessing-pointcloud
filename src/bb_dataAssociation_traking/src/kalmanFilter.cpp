#include "kalmanFilter.hpp"

KalmanFilter::KalmanFilter(double x_init, double y_init, int num_states, int num_sensors, 
                           const Eigen::MatrixXd& motion_model, 
                           const Eigen::MatrixXd& measurement_model,
                           const Eigen::MatrixXd& process_noise,
                            double a_cov, double xy_cov, double bb_cov, double cov_limit_factor) 
    : state_dim(num_states), sensor_dim(num_sensors),
      A(motion_model), H(measurement_model),R(process_noise),acell_cov_R_(a_cov), pose_cov_Q_(xy_cov),boundingBox_cov_Q_(bb_cov), covariance_limit_factor_(cov_limit_factor) {

    time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Identity matrix
    I = Eigen::MatrixXd::Identity(state_dim, state_dim);

    // Initialize state vector and covariance
    x = Eigen::VectorXd::Zero(state_dim);
    x_boundingBox = Eigen::VectorXd::Zero(state_dim);
    x[0]=x_init;
    x[1]=y_init;
    x_boundingBox[0]=x_init;
    x_boundingBox[1]=y_init;

    P = Eigen::MatrixXd::Identity(state_dim, state_dim);

    // Default process noise and measurement noise
    //R = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.5; //Sensor noise will be defined later on
    Q = Eigen::MatrixXd::Identity(sensor_dim, sensor_dim) * pose_cov_Q_; //MEASUREMENT NOISE

    // Override entries related to bounding box sizes with boundingBox_cov_Q
    Q(2, 2) = boundingBox_cov_Q_;
    Q(3, 3) = boundingBox_cov_Q_;
    
    // entries 2,3 describe the bounding box sizes measurements, we want these to have very high measurement noise and low process noite, these should be aprox constant


    // Create a mask matrix for replacing T and T²/2
    A_mask = Eigen::MatrixXd::Zero(state_dim, state_dim);
    for (int i = 0; i < state_dim; ++i) {
        for (int j = 0; j < state_dim; ++j) {
            if (A(i, j) == -1)      A_mask(i, j) = 1;  // Mark T
            else if (A(i, j) == -2) A_mask(i, j) = 2;  // Mark T²/2
        }
    }
    
}

void KalmanFilter::predict(rclcpp::Time currentTime) {

    double dt = (currentTime - time_).seconds();
    if (dt>10){
        dt=0.1;
    }
    if (dt ==0.00){
        std::cout<< "Time dif between cloud messages's stamp is null - no kf tracking is possible. Assuming dt=0.1s to keep running. currentTIME: "<< currentTime.seconds()<< " last time: "<<time_.seconds()<< "dt: "<< dt <<std::endl;
        dt=0.1;
    }
    std::cout << "currentTIME: "<< currentTime.seconds()<< " last time: "<<time_.seconds()<< "dt: "<< dt <<std::endl;
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
    Eigen::MatrixXd R_dt = R;
    for (int i = 0; i < state_dim; ++i) {
        for (int j = 0; j < state_dim; ++j) {
            if (R(i, j) == 4)      R_dt(i, j) = acell_cov_R_ * std::pow(dt,4)/4;        // Replace 4 with dt⁴/4
            else if (R(i, j) == 3) R_dt(i, j) = acell_cov_R_ * std::pow(dt,3)/2; // Replace 3 with dt³/2
            else if (R(i, j) == 2) R_dt(i, j) = acell_cov_R_ * std::pow(dt,2); // Replace 2 with dt²
        }
    }
    //INCREASE confidence that bouding box size will stay unchanged  - decreasing covariance of process noise of that
    R_dt(4, 4) = R_dt(4, 4)/2;
    R_dt(5, 5) = R_dt(5, 5)/2;
    R_dt(7, 7) = R_dt(7, 7)/2;
    R_dt(8, 8) = R_dt(8, 8)/2;

    R_dt(9, 9) = R_dt(9, 9)*2;
    R_dt(6, 6) = R_dt(6, 6)/2;


    // Predict step
    x = A_dt * x;
    P = A_dt * P * A_dt.transpose() + R_dt;
}



void KalmanFilter::update(const Eigen::VectorXd& z) {
    //std::cout << "updating  ";

    Eigen::VectorXd y = z - H * x;                 // Innovation
    Eigen::MatrixXd S = H * P * H.transpose() + Q; // Innovation covariance
    Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain
    //std::cout << "y:"<<y<< "  , K: "<< K << std::endl;
    // Update state
    x = x + K * y;
    P = (I - K * H) * P;

    //std::cout << "x in update: " << x << std::endl;
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
/*
     // Initialize the matrices
    process_noise <<    4,0, 3, 0,0,0,0, 0, 0, 0,//x
                        0,4, 0, 3,0,0,0, 0, 0, 0,//y
                        0,0, 2, 0,0,0,0, 0, 0, 0,//velx
                        0,0, 0, 2,0,0,0, 0, 0, 0,//vely
                        0,0, 0, 0,4,0,0, 3, 0, 0,//lengthBB
                        0,0, 0, 0,0,4,0, 0, 3, 0,//wigthBB
                        0,0, 0, 0,0,0,4, 0, 0, 3,//orientationBB
                        0,0, 0, 0,0,0,0, 2, 0, 0,//deltaLengthBB
                        0,0, 0, 0,0,0,0, 0, 2, 0,//deltaWidthBB
                        0,0, 0, 0,0,0,0, 0, 0, 2;//deltaOrientationBB*/
std::pair<double &, double &> KalmanFilter::produceBoundingBox_withCov(bool verbose){

    x_boundingBox = x;

    //99% confidense is 3 standar deviations
    Eigen::Matrix2d pos_cov; //includes covariance of x and y
    pos_cov << P(0,0), P(0,1),
        P(1,0), P(1,1);

    

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(pos_cov);
    Eigen::Vector2d eig_vals = eig.eigenvalues();
    Eigen::Matrix2d eig_vecs = eig.eigenvectors();
    // Get orientation of bounding box (theta)
    double theta = x[6]; // orientation in radians

    // Width and Length from state
    double width = x[5];  // along x' axis of the box
    double length = x[4]; // along y' axis of the box
     // Unit vectors along box's local axes (width = x', length = y')
    Eigen::Vector2d u(std::cos(theta), std::sin(theta));       // local x' (width)
    Eigen::Vector2d v(-std::sin(theta), std::cos(theta));      // local y' (length)

    // Project covariance along local box axes
    double var_u = u.transpose() * pos_cov * u;  // variance along width axis
    double var_v = v.transpose() * pos_cov * v;  // variance along length axis

    // Choose confidence multiplier: sqrt(5.991) ≈ 2.45 for 95% confidence ellipse
    // 95% confidence = 1.96=2 standard deviations
    double chi_squared_val = 5.991; // 2 DoF  
    double k = 2.45;

    // Inflate each dimension
    double inflated_bbwidth_withxy = width + k * std::sqrt(var_u);
    double inflated_bblength_withxy = length + k * std::sqrt(var_v) ;

    double width_axis_Elipselength = 2 * k * std::sqrt(var_u);
    double lenght_axis_Elipselength = 2 * k * std::sqrt(var_v);


    //length and width of bb are proporcional to the cov of position x,y and cov of lengthBB, wigthBB
    //                                                            x[0], x[1]        x[4],    x[5]
    //                                                          P[0,0], P[1,1]     P[4,4], P[5,5]              

  
    double std_length = std::sqrt(P(4,4));
    double std_width  = std::sqrt(P(5,5));

    if (verbose){
        std::cout << "pos_cov" << pos_cov << "\n bb cov" << std_length << " " << std_width << "\n";
    }

    double inflated_bblength_withWL = x[4] + k * std_length;
    double inflated_bbwidth_withWL  = x[5] + k * std_width;

    double total_inflated_bblength = inflated_bblength_withWL + k * std::sqrt(var_v);
    double total_inflated_bbwidth = inflated_bbwidth_withWL + k * std::sqrt(var_u);

    x_boundingBox[5]=total_inflated_bbwidth;
    x_boundingBox[4]=total_inflated_bblength;


    if (std::abs(total_inflated_bblength*total_inflated_bbwidth) > std::abs(width*length*covariance_limit_factor_) || total_inflated_bbwidth<x[5] ||total_inflated_bblength<x[4] ){
        std::cout << "A tracked object has too much covariance or neg covariance - we do not trust him. Not considered an object"<<std::endl;
        x_boundingBox[5]=0;
        x_boundingBox[4]=0;
        width_axis_Elipselength=0;
        lenght_axis_Elipselength=0;
    }
    return {width_axis_Elipselength, lenght_axis_Elipselength};

}
