#include "bounding_box_node.hpp"
#include "timing_metrics.cpp"
#include "laserscan_to_pointcloud.cpp"

// Define the global matrices
Eigen::MatrixXd motion_model;
Eigen::MatrixXd measurement_model;
Eigen::MatrixXd process_noise;


void initMatrices() {

    // Resize the matrices before using <<
    motion_model = Eigen::MatrixXd(num_states, num_states);
    measurement_model = Eigen::MatrixXd(num_sensors, num_states);
    process_noise = Eigen::MatrixXd(num_states, num_states);

        // Initialize the matrices
        //process noise R depends on dt. here 4 means entry is acell_cov_R_*(dt^4), 3 means acell_cov_R_*(dt^3) and 2 means acell_cov_R_*(dt^2)
    process_noise <<    4,0, 3, 0,0,0,0, 0, 0, 0,//x 0
                        0,4, 0, 3,0,0,0, 0, 0, 0,//y 1
                        0,0, 2, 0,0,0,0, 0, 0, 0,//velx 2 
                        0,0, 0, 2,0,0,0, 0, 0, 0,//vely 3
                        0,0, 0, 0,4,0,0, 3, 0, 0,//lengthBB 4
                        0,0, 0, 0,0,4,0, 0, 3, 0,//wigthBB 5
                        0,0, 0, 0,0,0,4, 0, 0, 3,//orientationBB 6
                        0,0, 0, 0,0,0,0, 2, 0, 0,//deltaLengthBB 7
                        0,0, 0, 0,0,0,0, 0, 2, 0,//deltaWidthBB 8
                        0,0, 0, 0,0,0,0, 0, 0, 2;//deltaOrientationBB 9


    /*   //in motion model -1 means the variable depends on other with relation to time, like the way x depends on velocity_x
    motion_model <<     1,0,-1, 0,0,0,0, 0, 0, 0,//x
                        0,1, 0,-1,0,0,0, 0, 0, 0,//y
                        0,0, 1, 0,0,0,0, 0, 0, 0,//velx
                        0,0, 0, 1,0,0,0, 0, 0, 0,//vely
                        0,0, 0, 0,1,0,0,-1, 0, 0,//lengthBB
                        0,0, 0, 0,0,1,0, 0,-1, 0,//wigthBB
                        0,0, 0, 0,0,0,1, 0, 0,-1,//orientationBB
                        0,0, 0, 0,0,0,0, 1, 0, 0,//deltaLengthBB
                        0,0, 0, 0,0,0,0, 0, 1, 0,//deltaWidthBB
                        0,0, 0, 0,0,0,0, 0, 0, 1;//deltaOrientationBB
    */ 
         //in motion model -1 means the variable depends on other with relation to time, like the way x depends on velocity_x
    motion_model <<     1,0,-1, 0,0,0,0, 0, 0, 0,//x
                        0,1, 0,-1,0,0,0, 0, 0, 0,//y
                        0,0, 1, 0,0,0,0, 0, 0, 0,//velx
                        0,0, 0, 1,0,0,0, 0, 0, 0,//vely
                        0,0, 0, 0,1,0,0,-1, 0, 0,//lengthBB
                        0,0, 0, 0,0,1,0, 0,-1, 0,//wigthBB
                        0,0, 0, 0,0,0,1, 0, 0,-1,//orientationBB
                        0,0, 0, 0,0,0,0, 0, 0, 0,//deltaLengthBB
                        0,0, 0, 0,0,0,0, 0, 0, 0,//deltaWidthBB
                        0,0, 0, 0,0,0,0, 0, 0, 0;//deltaOrientationBB
                    
    measurement_model <<1,0, 0, 0,0,0,0, 0, 0, 0,//x
                        0,1, 0, 0,0,0,0, 0, 0, 0,//y
                        0,0, 0, 0,1,0,0, 0, 0, 0,//lengthBB
                        0,0, 0, 0,0,1,0, 0, 0, 0,//wigthBB
                        0,0, 0, 0,0,0,1, 0, 0, 0;//orientationBB
    }

BoundingBoxNode::BoundingBoxNode() : Node("bounding_box_node")
    {
    
    initMatrices();
    cloud2D_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/clustered_points", 10, std::bind(&BoundingBoxNode::pointCloudCallback, this, std::placeholders::_1));

    bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bounding_boxes", 10);
    kf_bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/kalmanFilter/bounding_boxes", 10);

    corrected_bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bounding_boxes/corrected", 10);
    pub_NonTRacked_pc_ = this->create_publisher<sensor_msgs::msg::LaserScan>("static/laserscan", rclcpp::SensorDataQoS()); 



    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);

    RCLCPP_INFO( this->get_logger(), "Started pointcloud subscriber");
    last_iteration_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    
    if (save_metrics_txt_) { //s
        std::string base_filename = metrics_file;
        std::string final_filename = base_filename;

        if (fs::exists(base_filename + ".txt")) { //if tehre is already a file with this name, move on to file n+1
            int counter = 1;
            while (fs::exists(base_filename + std::to_string(counter) + ".txt")) {
                ++counter;
            }
            final_filename = base_filename + std::to_string(counter)+ ".txt";
        }

        outfile_.open(final_filename, std::ios::app);
        if (!outfile_.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to open %s for writing.", final_filename.c_str());
            save_metrics_txt_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Saving metrics to: %s", final_filename.c_str());
        }
    }
    if(saveTimeMetric_){
        timeoutFile_.open(timing_file, std::ios::app);
        //std::ofstream timeoutFile_(timing_file, std::ios::app); // append mode

        if (!timeoutFile_.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to open %s for writing.", timing_file.c_str());
            saveTimeMetric_ = false;
        } else{
            timeoutFile_ << "\n\nNode was restarted\n";

        }
            
    }




}


void BoundingBoxNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    ScopedTimer timer_calback("[bbTracking], Entire point cloud callback",this, timeMetric_,saveTimeMetric_,timeoutFile_ );
    ScopedTimer timer_transform("[bbTracking], Tranform cloud",this, timeMetric_,saveTimeMetric_, timeoutFile_);
    // Convert PointCloud2 to PCL PointCloud
    cout << "Begining callback"<<endl;
    pcl::PointCloud<pcl::PointXYZI> originalFrameCloud, cloud;
    pcl::fromROSMsg(*msg, originalFrameCloud);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(originalFrameCloud, originalFrameCloud, valid_indices);
    rclcpp::Time currentTime =rclcpp::Time(msg->header.stamp);
    if (currentTime.seconds() == 0) {
        rclcpp::Clock ros_clock(RCL_ROS_TIME);  // Use ROS time (sim time or system time depending on parameter)
        currentTime = ros_clock.now();
}    if(std::abs((currentTime - last_iteration_time_).seconds())>10){ //if more than 10 seconds of difference between callbacks, restart trackers
        trackedObjectsList.clear();
        currentObjectsList.clear();
    }

    if (originalFrameCloud.empty()) {
        std::cerr << "Point cloud is empty after removing NaNs. Skipping." << std::endl;
        cout << "Point cloud is empty after removing NaNs. Skipping." << endl;
        predictKalmanFilters(currentTime);
        pubKfMarkerArrays(fixed_frame_);
        return;
    }
    geometry_msgs::msg::TransformStamped transform_stamped, inverse_transform_stamped;

    if (fixed_frame_ != msg->header.frame_id){
        transform_stamped = tf2_->lookupTransform(fixed_frame_, msg->header.frame_id, tf2::TimePointZero);
        inverse_transform_stamped = tf2_->lookupTransform( msg->header.frame_id, fixed_frame_, tf2::TimePointZero);

            try{
                //pcl transform is much more effeciente than tf2::doTransform
                pcl_ros::transformPointCloud(originalFrameCloud, cloud, transform_stamped);

            }catch (...) {
                //there are some times that pcl transform fails and with floating point exception (with correct tfs, but probably bad points in the pointcloud),
                // in those cases, doing tf2::doTransform is needed
                std::cerr << "Transform lookup failed: ";//<< ex.what() << std::endl;
                sensor_msgs::msg::PointCloud2 cloud_out;

                tf2::doTransform(*msg, cloud_out, transform_stamped);
                pcl::fromROSMsg(cloud_out, cloud);

            }
    }else{
        cloud = originalFrameCloud;
        tf2::Transform tf2_transform;
        tf2_transform.setIdentity();
        tf2::toMsg(tf2_transform, inverse_transform_stamped.transform); //setting transform to identity/null transform

    }
    timer_transform.stopClock(); //prints out time


    

    //pcl::PointCloud<pcl::PointXYZI>::Ptr originalFrameCloud(new pcl::PointCloud<pcl::PointXYZI>), cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //predict next pose on kalman filters
    predictKalmanFilters(currentTime);
    last_iteration_time_=currentTime;
    ScopedTimer timer_dealClusters("[bbTracking], Separate Clusters",this,  timeMetric_,saveTimeMetric_,timeoutFile_ );


    // Separate clusters by intensity value
    // Separate clusters by intensity value - the clustering algorithm defines each cluster by a intensity value that all of its points have


    std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    for (const auto& point : cloud.points) {
        int cluster_id = static_cast<int>(point.intensity);
        if (clusters.find(cluster_id) == clusters.end()) {
            clusters[cluster_id] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        }
        clusters[cluster_id]->points.push_back(point);
    }

    //clusteres of zero intensity are non clustered points - this will be part of a sttaic map and not tracked
    if (!nonTrackedPc_) {
        nonTrackedPc_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    } else {
        nonTrackedPc_->clear();  // Make sure previous contents are removed
    }

    
    if (clusters.find(0) != clusters.end()) {
        *nonTrackedPc_ += *(clusters[0]);
    }
    clusters.erase(0);  //only points that are part of a cluster will be used to track objects

    timer_dealClusters.stopClock();
    ScopedTimer timer_obb("[bbTracking], Computing bounding boxes",this,  timeMetric_,saveTimeMetric_,timeoutFile_ );


    // Compute bounding boxes for each cluster
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    currentObjectsList.clear();
    for (const auto& cluster : clusters) {
        
        objectTracker object(cluster.second->points[0].x, cluster.second->points[0].y ); //initiate the object already at one point of the cluster, this helps the kf in the first few seconds
        visualization_msgs::msg::Marker marker;
        //pca2DBoundingBox(cluster.second, marker);
        object.rectangle = rotatingCaliper2DBoundingBox(cluster.second, marker);
        object.last_cluster = cluster.second;
        object.covInfo = computeCovarianceInfo(cluster.second);
        //define cost to all tracked objects in object.associationCosts

        defineCosts(object); //trackedObjectsList is a class variable and can be accessed inside. 

        marker.id = id++;
        marker.header.frame_id = fixed_frame_ ;
        marker_array.markers.push_back(marker);
        currentObjectsList.push_back(object);
        // Publish the instantaneous oriented bounding boxes 
    }
    bbox_pub_->publish(marker_array);

    //cout << "currentObjectsList.size()= " << currentObjectsList.size()<< endl;

    //cout << "trackedObjectsList.size()= " << trackedObjectsList.size()<< endl;
    timer_obb.stopClock();
    if (trackedObjectsList.size()>0){
        DataAssociate();    //HUngarian algorithm for matching - 
    } else{
        initiateTrackedObjects();//Will not do data associate, only iniciate
    }
    ScopedTimer timer_ukf("[bbTracking], Update kf",this,  timeMetric_,saveTimeMetric_,timeoutFile_ );

    updateKalmanFilters(); 
    pubKfMarkerArrays(fixed_frame_); //here we publish kf bounding boxes
    publishNonTrackedPC(msg->header.frame_id,currentTime,inverse_transform_stamped);
    pupBBMarkerArray(fixed_frame_); //here we publish the intantaneous oriented bounding boxes

    if(saveTimeMetric_){
        rclcpp::Clock ros_clock(RCL_ROS_TIME);  // Use ROS time (sim time or system time depending on parameter)
        rclcpp::Time MetriccurrentTime = ros_clock.now();
        double totalTime = (MetriccurrentTime - rclcpp::Time(msg->header.stamp)).seconds();
                timeoutFile_ <<  "[TotalTime],The total run time is " << std::fixed << std::setprecision(6) << totalTime << ", ms\n";
                std::cout<<  "[TotalTime],The total run time is " << std::fixed << std::setprecision(6) << totalTime << ", ms\n";

    }
   
    //computeMetrics(currentTime);//BAD METRICS
}

void BoundingBoxNode::pupBBMarkerArray(std::string frame_id){
// Compute bounding boxes for each cluster
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto& object : trackedObjectsList) {
        visualization_msgs::msg::Marker marker;
        marker.id = id++;
        marker.header.frame_id = frame_id ;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.pose.position.x = object.rectangle.center.x;
        marker.pose.position.y = object.rectangle.center.y;
        marker.pose.position.z = 0.0; // Z is ignored
        marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), object.rectangle.angle_width));
        marker.scale.x = object.rectangle.width;
        marker.scale.y = object.rectangle.height;
        marker.scale.z = 0.1; // Small height for 2D box
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5); // 0.5s
        marker_array.markers.push_back(marker);
        currentObjectsList.push_back(object);
         // Orientation arrow marker (ARROW)
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.id = id++;
        arrow_marker.header.frame_id = frame_id;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.pose.position.x = object.rectangle.center.x;
        arrow_marker.pose.position.y = object.rectangle.center.y;
        arrow_marker.pose.position.z = 0.05; // slightly above the cube
        arrow_marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), object.rectangle.angle_width));
        
        // Length of arrow in the direction of orientation
        arrow_marker.scale.x = 0.5 * object.rectangle.width;  // Arrow length
        arrow_marker.scale.y = 0.2 * object.rectangle.width; // Arrow shaft diameter
        arrow_marker.scale.z = 0.1; // Arrow head diameter
        
        arrow_marker.color.r = 0.50;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 0.50;
        arrow_marker.color.a = 1.0;
        arrow_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(arrow_marker);
    }
    corrected_bbox_pub_->publish(marker_array); 
}

CovarianceInfo BoundingBoxNode::computeCovarianceInfo(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    CovarianceInfo info;
    bool null_cov=false;

    if (cloud->empty()) {
        
        
        null_cov=true;

    }

    // Compute mean of x and y
    float mean_x = 0.0f, mean_y = 0.0f;
    for (const auto& point : cloud->points) {
        mean_x += point.x;
        mean_y += point.y;
    }
    mean_x /= cloud->size();
    mean_y /= cloud->size();
    info.meanxy(0) =mean_x;
    info.meanxy(1) = mean_y;

    // Initialize covariance matrix
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();

    for (const auto& point : cloud->points) {
        float dx = point.x - mean_x;
        float dy = point.y - mean_y;

        cov(0, 0) += dx * dx; // var(x)
        cov(0, 1) += dx * dy; // cov(x,y)
        cov(1, 0) += dx * dy; // cov(y,x)
        cov(1, 1) += dy * dy; // var(y)
    }

    cov /= static_cast<float>(cloud->size()-1);

    info.covariance = cov;
    info.determinant = cov.determinant();

    if (info.determinant != 0.0f) {
        info.inverse = cov.inverse();
    } else {
        null_cov=true;
        std::cout << "Matrix is not invertible!!" << std::endl;
    }

    if(null_cov){
        float large_value = std::numeric_limits<float>::max();

        info.covariance<< large_value, 0.0f,  large_value , 0.0f;//zeros to make it invertible
        info.determinant = info.covariance.determinant();
        info.inverse = info.covariance.inverse();
        info.meanxy = Eigen::Vector2f::Zero();
    }
    return info;


}
void BoundingBoxNode::publishNonTrackedPC(std::string frame_id, rclcpp::Time stamp, geometry_msgs::msg::TransformStamped transform_stamped){
    //point cloud made up of all points that do not belong to one of the tracked objects - will be part of a static map

    for(auto & currentObject : currentObjectsList){
        if (currentObject.newObject && currentObject.last_cluster && !currentObject.last_cluster->empty() && !currentObject.hasPublished_last_cluster) {
            // Append the cluster to the non-tracked point cloud
            *nonTrackedPc_ += *(currentObject.last_cluster);
            currentObject.hasPublished_last_cluster = true;

        }
        
    }
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud_transformed;
    try {
      
        pcl_ros::transformPointCloud(*nonTrackedPc_, pcl_cloud_transformed, transform_stamped); //we want to transfrom back from global frame to local frame
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(pcl_cloud_transformed, *cloud);
        cloud->header.frame_id = frame_id;
        cloud->header.stamp = stamp;
        auto laserscan = computeLaserScan(cloud);
        pub_NonTRacked_pc_->publish(std::move(laserscan));

    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform static cloud: %s", ex.what());
        return;
    }
    
    nonTrackedPc_->clear();

}


void BoundingBoxNode::computeMetrics(rclcpp::Time& currentTime){
    rclcpp::Duration metric_time_horizon_rclpp = rclcpp::Duration::from_seconds(metric_time_horizon);
    rclcpp::Time prediction_time = currentTime + metric_time_horizon_rclpp;
    for(auto& trackedObject : trackedObjectsList){
        Eigen::VectorXd predicted_state = trackedObject.kf.predictTheFuture(metric_time_horizon);
        trackedObject.future_predictions.push_back({prediction_time, predicted_state});
         // If the queue has fewer than minimum number of elements to achieve first prediction in current time
        if (trackedObject.future_predictions.size() < 10*metric_time_horizon)
         continue;
        auto best_match = getPrediction(trackedObject, currentTime);

        // Extract predicted center
        float predicted_x = best_match.predicted_state[0];
        float predicted_y =  best_match.predicted_state[1];

        // Current actual center
        float actual_x = trackedObject.rectangle.center.x;
        float actual_y = trackedObject.rectangle.center.y;

        // Compute RMSE between predicted and actual center
        float dx = predicted_x - actual_x;
        float dy = predicted_y - actual_y;
        float rmse = std::sqrt((dx * dx + dy * dy) / 2.0);

        // Extract predicted bbox
        float predicted_width =  best_match.predicted_state[5];
        float predicted_length =  best_match.predicted_state[4];
        float predicted_angle =  best_match.predicted_state[6];

        // Actual bbox
        float actual_width = trackedObject.rectangle.width;
        float actual_length = trackedObject.rectangle.height;
        float actual_angle = trackedObject.rectangle.angle_width;

        // Compute IoU of bounding boxes
        float iou = computeIoU(predicted_x, predicted_y, predicted_length, predicted_width, predicted_angle,
                            actual_x, actual_y, actual_length, actual_width, actual_angle);

        //std::cout << "Object Metrics: RMSE = " << rmse << " , IoU = " << iou << std::endl;
        //predict x tseconds in the future and add it to a queue inside the object struct

        //pop out last element in queu and compare that elelemt and the current estimate for this object
            //this comparisson is made with 2 metrics: Root mean squared error between center position and Intersection over Union (IoU) of the bouding boxes


    }

}

 
TimedPrediction BoundingBoxNode::getPrediction(objectTracker& object, rclcpp::Time& currentTime) {
    TimedPrediction best_match;
    double min_time_diff = std::numeric_limits<double>::max();
    int best_index = -1;

    // Find index of prediction closest to currentTime
    for (size_t i = 0; i < object.future_predictions.size(); ++i) {
        double time_diff = std::abs((object.future_predictions[i].predicted_time - currentTime).seconds());
        if (time_diff < min_time_diff) {
            min_time_diff = time_diff;
            best_match = object.future_predictions[i];
            best_index = static_cast<int>(i);
        }
    }

    // Remove everything before best match
    if (best_index > 0) {
        object.future_predictions.erase(object.future_predictions.begin(), 
                                        object.future_predictions.begin() + best_index);
    }

    return best_match;
}

float BoundingBoxNode::computeIoU(float x1, float y1, float len1, float wid1, float angle1,
    float x2, float y2, float len2, float wid2, float angle2) {
    cv::RotatedRect rect1(cv::Point2f(x1, y1), cv::Size2f(len1, wid1), angle1* 180.0f / CV_PI);
    cv::RotatedRect rect2(cv::Point2f(x2, y2), cv::Size2f(len2, wid2), angle2* 180.0f / CV_PI);

    float intersection_area = 0.0f;
    std::vector<cv::Point2f> intersection_pts;
    int result = cv::rotatedRectangleIntersection(rect1, rect2, intersection_pts);
    if (result == cv::INTERSECT_PARTIAL || result == cv::INTERSECT_FULL) {
        intersection_area = (float)cv::contourArea(intersection_pts);
    }

    float area1 = len1 * wid1;
    float area2 = len2 * wid2;
    float union_area = area1 + area2 - intersection_area;
    //std::cout << "intersection_area" <<intersection_area << " union_area:"<< union_area << "area1"<< area1 << std::endl;

    if (union_area > 0.0f) {
            return intersection_area / union_area;
        } else {
            return 0.0f;
        }
}

void BoundingBoxNode::pubKfMarkerArrays(std::string frame_id){
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for(auto & trackedObject : trackedObjectsList){           

        double velocity = std::sqrt(pow(trackedObject.kf.x[2],2) + pow(trackedObject.kf.x[3],2));
        if((!trackedObject.newObject) && (velocity > min_velocity_threshold_)){
            auto [elipse_width, elipse_length] =trackedObject.kf.produceBoundingBox_withCov(trackedObject.id == 0 ||trackedObject.id==1 );
            if(save_metrics_txt_){
                saveMetricsTxt(trackedObject);
            }
            Eigen::VectorXd state = trackedObject.kf.x;
            Eigen::VectorXd augmented_state = trackedObject.kf.x_boundingBox;

            
            //draw marker elipse with heading of heading state[6] and [elipse_width, elipse_lenght]
            visualization_msgs::msg::Marker marker_elip;
            marker_elip.type = visualization_msgs::msg::Marker::CYLINDER;
            marker_elip.header.frame_id = frame_id;
            marker_elip.id = id++;
            marker_elip.pose.position.x = state[0];
            marker_elip.pose.position.y = state[1];
            marker_elip.pose.position.z = 0.05; // Slightly above ground

            // Align with heading
            tf2::Quaternion q;
            q.setRPY(0, 0, state[6]); // Z-axis rotation
            marker_elip.pose.orientation = tf2::toMsg(q);

            marker_elip.scale.x = elipse_width;   // minor axis (width)
            marker_elip.scale.y = elipse_length;  // major axis (length)
            marker_elip.scale.z = 0.01;           // Flat ellipse

            marker_elip.color.r = 0.0;
            marker_elip.color.g = 0.0;
            marker_elip.color.b = 0.80;
            marker_elip.color.a = 0.4;

            marker_elip.lifetime = rclcpp::Duration::from_seconds(1);
            marker_array.markers.push_back(marker_elip);

            //draw maker cube of trackedObject.kf.x_boundingBox - with its width and its height
            visualization_msgs::msg::Marker marker_bigbb;
            marker_bigbb.type = visualization_msgs::msg::Marker::CUBE;
            marker_bigbb.pose.position.x = state[0];
            marker_bigbb.pose.position.y = state[1];
            marker_bigbb.pose.position.z = 0.2; // Z is ignored
            marker_bigbb.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), state[6]));
            marker_bigbb.scale.x = augmented_state[5]; // width
            marker_bigbb.scale.y = augmented_state[4]; // length = height
            marker_bigbb.scale.z = 0.1; // Small height for 2D box
            marker_bigbb.color.r = 0.4;
            marker_bigbb.color.g = 0.0;
            marker_bigbb.color.b = 0.0;
            marker_bigbb.color.a = 0.3;
            marker_bigbb.id = id++;
            marker_bigbb.header.frame_id = frame_id;
            marker_bigbb.lifetime = rclcpp::Duration::from_seconds(1); // 0.5s
            marker_array.markers.push_back(marker_bigbb);

            
            visualization_msgs::msg::Marker marker;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.pose.position.x = state[0];
            marker.pose.position.y = state[1];
            marker.pose.position.z = 0.0; // Z is ignored
            marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), state[6]));
            marker.scale.x = state[5]; // width
            marker.scale.y = state[4]; // length = height
            marker.scale.z = 0.1; // Small height for 2D box
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.6;
            marker.id = id++;
            marker.header.frame_id = frame_id;
            marker.lifetime = rclcpp::Duration::from_seconds(1); // 0.5s
            marker_array.markers.push_back(marker);

            // Velocity Arrow Marker
            visualization_msgs::msg::Marker arrow_marker;
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.header.frame_id = frame_id;
            arrow_marker.id = id++;
            arrow_marker.scale.x = state[5]/8; // Shaft diameter
            arrow_marker.scale.y = state[5]/4;  // Arrowhead diameter
            arrow_marker.scale.z = 0.0;  // Not used for arrows
            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 1.0; // Blue arrow
            arrow_marker.color.a = 0.80; // Fully visible

            geometry_msgs::msg::Point start, end;
            start.x = state[0];
            start.y = state[1];
            start.z = 0.1;
            end.x = state[0] + state[2]; // velx
            end.y = state[1] + state[3]; // vely
            end.z = 0.1;

            arrow_marker.points.push_back(start);
            arrow_marker.points.push_back(end);
            arrow_marker.lifetime = rclcpp::Duration::from_seconds(1);
            marker_array.markers.push_back(arrow_marker);

            // ID Text Marker
            visualization_msgs::msg::Marker text_marker;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.header.frame_id = frame_id;
            text_marker.id = id++;
            text_marker.pose.position.x = state[0];
            text_marker.pose.position.y = state[1];
            text_marker.pose.position.z = 0.3; // Slightly above the bounding box
            text_marker.scale.z = std::max(0.2, 0.5 * std::max(state[4], state[5])); // Proportional text size
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = "id: " + std::to_string(trackedObject.id);
            text_marker.lifetime = rclcpp::Duration::from_seconds(1);
            marker_array.markers.push_back(text_marker);

            

        }else if(!trackedObject.hasPublished_last_cluster){
            //send this objects cluster to static map if he is not being shown as to be tracked
            *nonTrackedPc_ += *(trackedObject.last_cluster);
            trackedObject.hasPublished_last_cluster = true;
        }
        
    }

    kf_bbox_pub_->publish(marker_array);
}


void  BoundingBoxNode::saveMetricsTxt(const objectTracker& trackedObject){
    // Get current time in seconds (as frame value), at this functions' call, last_iteration_time_ is already equal to currentTime
    double currentTimeSec = last_iteration_time_.seconds();  // Don't cast to int

    // Get the bounding box state
    Eigen::VectorXd augmented_state = trackedObject.kf.x_boundingBox;

    // Extract MOT16 parameters
    int id = trackedObject.id;
    double center_x = augmented_state[0];
    double center_y = augmented_state[1];
    double width    = augmented_state[5];
    double height   = augmented_state[4];

    // Convert center to top-left
    double bb_left = center_x - width / 2.0;
    double bb_top  = center_y - height / 2.0;

    double conf = 1.0; // Confidence (if unknown, set to 1.0)
    double x = center_x, y = center_y, z = 0.0; //unused
    
    // Write in format to then addapt to MOT16 
    outfile_ << std::fixed << std::setprecision(2) << currentTimeSec << ","
            << std::fixed << std::setprecision(0)<< id << ","
            << std::fixed << std::setprecision(2) //defines 2 decimal places as precision
            << bb_left << "," << bb_top << "," 
            << width << "," << height << ","                    // velx                             vely
            << conf << "," << x << "," << y << "," << z << "," << trackedObject.kf.x[2]  << "," << trackedObject.kf.x[3] <<"\n";

}


void BoundingBoxNode::initiateTrackedObjects(){
    //to deal with new objects 
    for(auto const& object : currentObjectsList){
        trackedObjectsList.push_back(object);//this newly created object already comes with .updateStepKF=true, newObject = true;newObjectCounter = 0;
    }    
}


void BoundingBoxNode::updateKalmanFilters(){
    Eigen::VectorXd z(num_sensors);
    for(auto& trackedObject : trackedObjectsList){
        if (trackedObject.updateStepKF){
            //cout << "updating a kf";
            /*if (trackedObject.rectangle.width>trackedObject.rectangle.height){
                std::swap(trackedObject.rectangle.width,trackedObject.rectangle.height);
                trackedObject.rectangle.angle_width =  trackedObject.rectangle.angle_height;    //trackedObject.rectangle.angle_width-M_PI/2; //need to fix this
                }*/
            correctBBorientation(trackedObject);
            z << trackedObject.rectangle.center.x, trackedObject.rectangle.center.y,  trackedObject.rectangle.height, trackedObject.rectangle.width, trackedObject.rectangle.angle_width;
            //cout << z<<endl;
            trackedObject.kf.update(z);
        }
    }

}

void BoundingBoxNode::correctBBorientation(objectTracker& trackedObject){
    //logic that longest bounding box side needs to point with angle inferior to 90 degrees to velocity direction
    double velx = trackedObject.kf.x[2];
    double vely = trackedObject.kf.x[3];
    double angle_kf=std::atan2(vely, velx);
    double angle_bb = trackedObject.rectangle.angle_width;
    double dif_angle= angle_kf-angle_bb;
    // Normalize angle difference to [-pi, pi]
    while (dif_angle > M_PI) dif_angle -= 2 * M_PI;
    while (dif_angle < -M_PI) dif_angle += 2 * M_PI;



    if(std::abs(dif_angle)<= M_PI/4 ){ //up to 45 degrees
        // Already aligned, do nothing
    }else if(std::abs(dif_angle)< 3*M_PI/4){ //from 45 up to 135 degrees the length and width of bb needs to be changed,the angle_bb needs to be added or subtracted 90 degrees, whichever makes it closest to angle_kf
        std::swap(trackedObject.rectangle.height, trackedObject.rectangle.width);
        if( std::abs(angle_kf-(angle_bb + M_PI/2))< std::abs(angle_kf-(angle_bb - M_PI/2))  ){
            trackedObject.rectangle.angle_width += M_PI/2;
        }else{
            trackedObject.rectangle.angle_width -= M_PI/2;
        }

    }else if(std::abs(dif_angle)< M_PI){ //from 135 to 180 degrres, width and length are kept, M_pi added to the angle_bb - effectively facing bb the other way around
        if(angle_bb>angle_kf){
            trackedObject.rectangle.angle_width -= M_PI;
        }
        else{
            trackedObject.rectangle.angle_width += M_PI;
        }
    }
}
void BoundingBoxNode::DataAssociate(){
    ScopedTimer timer_dataAss("[bbTracking], Data Association",this,  timeMetric_,saveTimeMetric_,timeoutFile_ );


    costMatrix_.clear();
    assignment_.clear(); //HungAlgo.solve already does this, but this way is easier to read the code
    int it=0;
    for(auto const& currentObject : currentObjectsList){
       costMatrix_.push_back(currentObject.costVector);
        it++;
        /*// Print costMatrix_
        cout<< "New row in costMatrix_:";
        //std::ostringstream oss;
        for (const auto& value : currentObject.costVector) {
            cout << value;
        }*/
        
    } 
    //cost matrix
    /*      old tracked objects 1,2,3 
                        {       { , , }     object 1 in this frame
                                { , , }     object 2 in this frame
                                { , , }     object 3 in this frame
                                { , , }    }object 4 in this frame
    */
    // assigment_ is a vector of indices int. It has cost matrix.Nrows size
    //assigment corresponds in indexes to each old tracked object, to the row chosen

    //cout << costMatrix_ << "asssignment: "<< assignment_ << endl;
    double cost = HungAlgo.Solve(costMatrix_, assignment_);
    unsigned int nRows = costMatrix_.size(); //nr objects in this frame
	unsigned int nCols = costMatrix_[0].size(); //nr objects tracked
    /*cout << "rows, coluns: "<< nRows<< nCols;
    cout << endl<< "Assignment matrix: ";
    for (auto aux : assignment_){
        cout << aux << ", ";
    }*/
    for (size_t it = 0; it < currentObjectsList.size(); it++) {
        objectTracker& currentObject = currentObjectsList[it];
        int assignedObject= assignment_[it];
        //The hungarian algorithm does not allow for negative costs to represent objects we do not want to associate with eachother
        //This means that associations above a cost_threshold will just be discarted
        if (costMatrix_[it][assignedObject]> cost_threshold_){
            assignment_[it]=-1; //this is done this way, because it is helpfull in finMissingNumbers
            assignedObject= assignment_[it];
        }
      
        if (assignedObject>=0 && assignedObject< nCols){
            objectTracker& trackedObject = trackedObjectsList[assignedObject];
            trackedObject.updateStepKF=true;
            trackedObject.rectangle= currentObject.rectangle;   //bouding box rectangle has all the needed information to update KF
            trackedObject.covInfo = currentObject.covInfo;      //covariance of the last point cloud cluster - used in next time step for data association
            trackedObject.last_cluster = currentObject.last_cluster;
            if (trackedObject.newObject) {
                trackedObject.newObjectCounter++;
                if (trackedObject.newObjectCounter >trackedObject.newObjectThreshold){ //contiditon to add object
                        trackedObject.id=id_counter_;
                        id_counter_ +=1;
                        trackedObject.newObject=false;
                        trackedObject.hasPublished_last_cluster = false; //will make it so that last cluster is added to published static pointcloud by this node

                }
            trackedObject.ocludedCounter=0; //reset on the oclusion counter
            }
        }
        else {
            //Creating a new object - it will only be published if it keeps apearing for some time
            trackedObjectsList.push_back(currentObject);//this newly created object already comes with .updateStepKF=true, newObject = true;newObjectCounter = 0;
        
        }
    }
    
    
    //to deal with OLD objects that were not associated to any NEW objects:
    std::vector<int> missing = findMissingNumbers(assignment_, nCols);
    //cout << "missing matrix:" ;
    for (int num : missing) {
        objectTracker& trackedObject = trackedObjectsList[num];
        // no match for this old tracked object
        trackedObject.updateStepKF=false;
        trackedObject.ocludedCounter++;
        //cout << num << " , ";
        if (trackedObject.ocludedCounter>trackedObject.pruneThreshold){ //condition to prune
            //lets not track this object anymore
            trackedObjectsList.erase(trackedObjectsList.begin()+num); //erasing is possible, as the missing indices are sorted in descending order
            //cout << endl<< "I HAVE ERASED AN OBJECT NOW" <<endl<<endl;
        }   
        
        }    
    //cout << endl;
  
        //trackedObjectsList.push_back(currentObjectsList[num])
}
    
        
std::vector<int> BoundingBoxNode::findMissingNumbers(const std::vector<int>& assignment, int m) {
    std::vector<bool> present(m, false); // Track which numbers are in assignment
    std::vector<int> missing;

    // Mark numbers that appear in assignment
    for (int num : assignment) {
        if (num >= 0 && num < m) {  // Ensure index is within bounds
            present[num] = true;
        } 
    }

    // Collect numbers that are missing
    for (int i = 0; i < m; ++i) {  // Fix off-by-one error
        if (!present[i]) {
            missing.push_back(i);
        }
    }
    std::sort(missing.rbegin(), missing.rend()); // sort in descending order

    return missing;
}


void BoundingBoxNode::defineCosts(objectTracker& object){
    object.costVector.clear();

    for(auto const& trackedObject : trackedObjectsList){
        double aux_var= costFuntion_VANILA(object, trackedObject);
        if (aux_var<cost_threshold_){
            object.costVector.push_back(aux_var);
        } else{ //this association will get rejected either way after the hungarian algorithm runs so we will
            object.costVector.push_back(aux_var*10000); //make it so big that it does not affect other associations
        }
    }
}

double BoundingBoxNode::costFuntion_VANILA(const objectTracker& object, const objectTracker& trackedObject){


    return std::sqrt(  std::pow((object.rectangle.center.x - trackedObject.rectangle.center.x),2) + std::pow((object.rectangle.center.y - trackedObject.rectangle.center.y),2)  );
}

double BoundingBoxNode::costFuntion_IOU(const objectTracker& object, const objectTracker& trackedObject){

    // Compute IoU of bounding boxes
    double iou = computeIoU(trackedObject.rectangle.center.x, trackedObject.rectangle.center.y, trackedObject.rectangle.height,  trackedObject.rectangle.width, trackedObject.rectangle.angle_width,
                        object.rectangle.center.x, object.rectangle.center.y, object.rectangle.height,  object.rectangle.width, object.rectangle.angle_width);
    if (iou>0.05){
        return (1.0/iou) - 1.0; //cost goes from 0 to 19
    }else{
        return 20; //bad cost for not interseting
    }
                        

}

double BoundingBoxNode::costFuntion_BACHY(const objectTracker& object, const objectTracker& trackedObject){

    //this equation is equation (5) from the paper https://www.sciencedirect.com/science/article/pii/S0029801823003232#sec3.2.2

    /*struct CovarianceInfo {
    Eigen::Matrix2f covariance;
    Eigen::Matrix2f inverse;
    Eigen::Vector2f meanxy;
    float determinant;
    };*/

    //trackedObject.covInfo already has mean updated with predicted step of kf, covariance is the cov from the last cluster
    //object.covInfo

    Eigen::Vector2f mean_dif= object.covInfo.meanxy - trackedObject.covInfo.meanxy;
    Eigen::Matrix2f cov_sum= 0.5*(object.covInfo.covariance + trackedObject.covInfo.covariance);
    float means_var;
    if(cov_sum.determinant()!=0.0f){
        means_var = 0.125* mean_dif.transpose() *cov_sum.inverse() *mean_dif;
    } else{
        means_var = 0.125* mean_dif.transpose() *mean_dif; //replace the inverse by the identity, in a 2x2 case, we can leave the inverse out
    }

    float log_cov = std::log(cov_sum.determinant()/std::sqrt(object.covInfo.determinant * trackedObject.covInfo.determinant ));

    return static_cast<double>(0.5*log_cov + means_var);

}

double BoundingBoxNode::costFuntion_BACHY_covBB(const objectTracker& object, const objectTracker& trackedObject){

    // trackedObject.covInfo already has mean updated with predicted step of kf
    // This function is similar to costFuntion_BACHY but considers the covariance fo the tracked object to be the cov from its kf

    Eigen::Matrix2f trackedObject_covInfo_covariance = approximateCovarianceFromBoundingBox(trackedObject.kf.x[4], trackedObject.kf.x[5] , trackedObject.kf.x[6]);
    //we cannot use the covariance of the kf, because that estimates how precise our kf is and does not make estimates about our object size itself
    //the covariance of a cluster of points evaluates how much these points divide themselfs in space, for this reason, the kf estimate for width and height transformed into aligned xy coordinates
    // is used as an aproximation - Use half the width/length of the bounding box aligned with xy axis as standard deviations, then square them to get variances

    float trackedObject_covInfo_determinant=trackedObject_covInfo_covariance.determinant();

    Eigen::Vector2f mean_dif= object.covInfo.meanxy - trackedObject.covInfo.meanxy;
    Eigen::Matrix2f cov_sum= 0.5*(object.covInfo.covariance + trackedObject_covInfo_covariance);
    float means_var;
    if(cov_sum.determinant()!=0.0f){
        means_var = 0.125* mean_dif.transpose() *cov_sum.inverse() *mean_dif;
    } else{
        means_var = 0.125* mean_dif.transpose() *mean_dif; //replace the inverse by the identity, in a 2x2 case, we can leave the inverse out
    }

    float log_cov = std::log(cov_sum.determinant()/std::sqrt(object.covInfo.determinant * trackedObject_covInfo_determinant ));

    return static_cast<double>(0.5*log_cov + means_var);
}
double BoundingBoxNode::costFuntion_BACHY_IOU(const objectTracker& object, const objectTracker& trackedObject){

    double w = 0.2;
    auto iou = costFuntion_IOU(object,trackedObject);
    auto Bhattacharyya_distance =costFuntion_BACHY(object,trackedObject);
    std::cout << "Iou" << iou << " Bhattacharyya_distance:" <<Bhattacharyya_distance<< std::endl;
    return iou*w + Bhattacharyya_distance;
}
double BoundingBoxNode::costFuntion_BACHY_IOU_eucledian(const objectTracker& object, const objectTracker& trackedObject){

    double w = 0.2; 
    double m_ = 2.0;
    auto iou = costFuntion_IOU(object,trackedObject); // 1/IOU - penalizes bad iou
    auto eucledian = costFuntion_VANILA(object,trackedObject);
    auto Bhattacharyya_distance =costFuntion_BACHY(object,trackedObject);
    std::cout << "Iou" << iou << " Bhattacharyya_distance:" <<Bhattacharyya_distance<< "eucledian"<<eucledian<< std::endl;
    return iou*w + Bhattacharyya_distance + eucledian*m_ ;
}


Eigen::Matrix2f BoundingBoxNode::approximateCovarianceFromBoundingBox(float width, float length, float heading) {
    //Use half the width/length of the bounding box aligned with xy axis as standard deviations, then square them to get variances
    
    // Standard deviations as half of dimensions (heuristic)
    float sigma_x = length / 2.0f;
    float sigma_y = width / 2.0f;

    // Rotation matrix from heading
    float cos_theta = std::cos(heading);
    float sin_theta = std::sin(heading);
    Eigen::Matrix2f R;
    R << cos_theta, -sin_theta,
         sin_theta,  cos_theta;

    // Diagonal covariance in local frame
    Eigen::Matrix2f local_cov = Eigen::Matrix2f::Zero();
    local_cov(0, 0) = sigma_x * sigma_x;
    local_cov(1, 1) = sigma_y * sigma_y;

    // Rotate to global frame
    Eigen::Matrix2f global_cov = R * local_cov * R.transpose();
    return global_cov;
}
void BoundingBoxNode::predictKalmanFilters(rclcpp::Time currentTime){
    ScopedTimer timer_predictKf("[bbTracking], predict KF",this,  timeMetric_,saveTimeMetric_,timeoutFile_ );

    for(auto& trackedObject : trackedObjectsList){
        trackedObject.kf.predict(currentTime);
        trackedObject.covInfo.meanxy(0)= trackedObject.kf.x[0];
        trackedObject.covInfo.meanxy(1)=trackedObject.kf.x[1];
    }


}


void BoundingBoxNode::pca2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker) {
    if (cloud->empty()) return;

    // Compute centroid
    Eigen::Vector2f centroid(0.0f, 0.0f);
    for (const auto& point : cloud->points) {
        centroid += Eigen::Vector2f(point.x, point.y);
    }
    centroid /= cloud->size();

    // Compute covariance matrix
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const auto& point : cloud->points) {
        Eigen::Vector2f diff(point.x - centroid.x(), point.y - centroid.y());
        covariance += diff * diff.transpose();
    }
    covariance /= cloud->size();

    // Compute PCA eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    Eigen::Matrix2f eigenVectors = solver.eigenvectors();

    // Transform points into PCA-aligned space
    Eigen::Matrix2f transform = eigenVectors.transpose();
    Eigen::Vector2f minPt(FLT_MAX, FLT_MAX), maxPt(-FLT_MAX, -FLT_MAX);
    
    for (const auto& point : cloud->points) {
        Eigen::Vector2f projected = transform * (Eigen::Vector2f(point.x, point.y) - centroid);
        minPt = minPt.cwiseMin(projected);
        maxPt = maxPt.cwiseMax(projected);
    }

    // Compute bounding box center and orientation
    Eigen::Vector2f bboxCenter = (minPt + maxPt) * 0.5f;
    Eigen::Vector2f worldCenter = eigenVectors * bboxCenter + centroid;
    float angle = std::atan2(eigenVectors(1, 0), eigenVectors(0, 0));

    // Create marker for visualization
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = worldCenter.x();
    marker.pose.position.y = worldCenter.y();
    marker.pose.position.z = 0.0; // Z is ignored
    marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), angle));
    marker.scale.x = maxPt.x() - minPt.x();
    marker.scale.y = maxPt.y() - minPt.y();
    marker.scale.z = 0.1; // Small height for 2D box
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;
    marker.lifetime = rclcpp::Duration::from_seconds(0); // 0.5s
}


std::vector<Point> BoundingBoxNode::convertPCLCloudToCalipersInput(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    std::vector<Point> pts;

    for (const auto& pcl_point : cloud->points) {
        Point pt;
        pt.x = pcl_point.x;
        pt.y = pcl_point.y;
        pts.push_back(pt);
    }

    return pts;
}

MinAreaRect BoundingBoxNode::rotatingCaliper2DBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, visualization_msgs::msg::Marker& marker) {

    auto pts = BoundingBoxNode::convertPCLCloudToCalipersInput(cloud);
    MinAreaRect res = RotatingCalipers::minAreaRect(pts);

    /*struct MinAreaRect
    {
    double width;
    double height;
    double area;
    double angle_width;
    double angle_height;
    Point center;
    Point corner;
    Point vector_width;
    Point vector_height;
    vector<Point> rectt
    };
    angles exemples:
width:0.308996, height:0.785398
width:0.997131, height:0.785398
width:0.546987, height:0.785398
width:0.888812, height:0.785398
width:0.509868, height:0.785398
width:0.0891392, height:0.785398
width:0.993217, height:0.785398
width:0.0478368, height:0.785398
width:0.333497, height:0.785398
width:0.153372, height:0.785398
width:0.257887, height:0.785398
width:1.02645, height:0.785398
width:1.17346, height:0.785398
width:1.43603, height:0.785398

    */
    // Create marker for visualization
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = res.center.x;
    marker.pose.position.y = res.center.y;
    marker.pose.position.z = 0.0; // Z is ignored
    marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), res.angle_width));
    marker.scale.x = res.width;
    marker.scale.y = res.height;
    marker.scale.z = 0.1; // Small height for 2D box
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5); // 0.5s
    return res;
}
BoundingBoxNode::~BoundingBoxNode() {
    if (outfile_.is_open()) {//this deals with the metrics txt file - needs to be closed
        outfile_.close();
    }
    if(timeoutFile_.is_open()){
        timeoutFile_.close();
    }
}