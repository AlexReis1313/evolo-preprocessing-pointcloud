#include "bounding_box_node.hpp"


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
    process_noise <<    4,0, 3, 0,0,0,0, 0, 0, 0,//x
                        0,4, 0, 3,0,0,0, 0, 0, 0,//y
                        0,0, 2, 0,0,0,0, 0, 0, 0,//velx
                        0,0, 0, 2,0,0,0, 0, 0, 0,//vely
                        0,0, 0, 0,4,0,0, 3, 0, 0,//lengthBB
                        0,0, 0, 0,0,4,0, 0, 3, 0,//wigthBB
                        0,0, 0, 0,0,0,4, 0, 0, 3,//orientationBB
                        0,0, 0, 0,0,0,0, 2, 0, 0,//deltaLengthBB
                        0,0, 0, 0,0,0,0, 0, 2, 0,//deltaWidthBB
                        0,0, 0, 0,0,0,0, 0, 0, 2;//deltaOrientationBB


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
                    
    measurement_model <<1,0, 0, 0,0,0,0, 0, 0, 0,//x
                        0,1, 0, 0,0,0,0, 0, 0, 0,//y
                        0,0, 0, 0,1,0,0, 0, 0, 0,//lengthBB
                        0,0, 0, 0,0,1,0, 0, 0, 0,//wigthBB
                        0,0, 0, 0,0,0,1, 0, 0, 0;//orientationBB
    }

BoundingBoxNode::BoundingBoxNode() : Node("bounding_box_node") {
    
    initMatrices();
    cloud2D_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/clustered_points", 10, std::bind(&BoundingBoxNode::pointCloudCallback, this, std::placeholders::_1));

    bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bounding_boxes", 10);
    kf_bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/kalmanFilter/bounding_boxes", 10);

    if (draw_height_){
    cloud3D_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rotated_pointcloud", rclcpp::SensorDataQoS().keep_last(2), std::bind(&BoundingBoxNode::pointCloud3DBuffer, this, std::placeholders::_1));
    }
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);

    RCLCPP_INFO(
        this->get_logger(),
        "Started pointcloud subscriber");


}


void BoundingBoxNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert PointCloud2 to PCL PointCloud
    cout << "Begining callback"<<endl;
    geometry_msgs::msg::TransformStamped transform_stamped;

    pcl::PointCloud<pcl::PointXYZI> originalFrameCloud, cloud;

    transform_stamped = tf2_->lookupTransform(fixed_frame_, msg->header.frame_id, tf2::TimePointZero);
    
    cout << "going to transform cloud"<<endl;

    //pcl::PointCloud<pcl::PointXYZI>::Ptr originalFrameCloud(new pcl::PointCloud<pcl::PointXYZI>), cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, originalFrameCloud);

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(originalFrameCloud, originalFrameCloud, valid_indices);

    if (originalFrameCloud.empty()) {
        std::cerr << "Point cloud is empty after removing NaNs. Skipping." << std::endl;
        cout << "Point cloud is empty after removing NaNs. Skipping." << endl;
        return;
    }

    cout << "step1"<<endl;

    try{
        //pcl transform is much more effeciente than tf2::doTransform
        pcl_ros::transformPointCloud(originalFrameCloud, cloud, transform_stamped);

    }catch (...) {
        //there are some times that pcl transform fails and with floating point exception (with correct tfs, but probably bad points in the pointcloud),
        // in those cases, doing tf2::doTransform is needed
        std::cerr << "Transform lookup failed: ";//<< ex.what() << std::endl;
        cout << "/n/n/n CATCHING ERROR /n/n/n/n";
        sensor_msgs::msg::PointCloud2 cloud_out;

        tf2::doTransform(*msg, cloud_out, transform_stamped);
        pcl::fromROSMsg(cloud_out, cloud);
    }

    cout << "going to get time"<<endl;

    double currentTime=rclcpp::Time(msg->header.stamp).seconds();
    //predict next pose on kalman filters
    cout << "going to predict kf"<<endl;

    predictKalmanFilters(currentTime);
    cout << "going to separate clusters"<<endl;

    // Separate clusters by intensity value
    std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    for (const auto& point : cloud.points) {
        int cluster_id = static_cast<int>(point.intensity);
        if (clusters.find(cluster_id) == clusters.end()) {
            clusters[cluster_id] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        }
        clusters[cluster_id]->points.push_back(point);
    }
    cout << "going to compute bb"<<endl;

    // Compute bounding boxes for each cluster
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    currentObjectsList.clear();
    for (const auto& cluster : clusters) {
        objectTracker object;
        visualization_msgs::msg::Marker marker;
        //pca2DBoundingBox(cluster.second, marker);
        object.rectangle = rotatingCaliper2DBoundingBox(cluster.second, marker);
        object.last_cluster = cluster.second;

        //define cost to all tracked objects in object.associationCosts
        defineCosts(object); //trackedObjectsList is a class variable and can be accessed inside. 

        if(draw_height_){
            defineHeight(object);
            marker.scale.z = object.height; //define height of the box
            marker.pose.position.z = object.height/2;
        }
        marker.id = id++;
        marker.header.frame_id = fixed_frame_ ;
        marker_array.markers.push_back(marker);
        currentObjectsList.push_back(object);
    }
    cout << "currentObjectsList.size()= " << currentObjectsList.size()<< endl;

    cout << "trackedObjectsList.size()= " << trackedObjectsList.size()<< endl;

    if (trackedObjectsList.size()>0){
        cout << "Will now data associate"<<endl;
        DataAssociate();    //HUngarian algorithm for matching - 
    } else{
        cout << "Will not do data associate, only iniciate"<<endl;
        initiateTrackedObjects();
    }

    updateKalmanFilters();
    pubKfMarkerArrays(fixed_frame_);
  
    computeMetrics(currentTime);

    // Publish the bounding boxes
    bbox_pub_->publish(marker_array);

}

void BoundingBoxNode::computeMetrics(double& currentTime){
    
    double prediction_time = currentTime + metric_time_horizon;
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

        std::cout << "Object Metrics: RMSE = " << rmse << " , IoU = " << iou << std::endl;
        //predict x tseconds in the future and add it to a queue inside the object struct

        //pop out last element in queu and compare that elelemt and the current estimate for this object
            //this comparisson is made with 2 metrics: Root mean squared error between center position and Intersection over Union (IoU) of the bouding boxes


    }

}

 
TimedPrediction BoundingBoxNode::getPrediction(objectTracker& object, double& currentTime) {
    TimedPrediction best_match;
    double min_time_diff = std::numeric_limits<double>::max();
    int best_index = -1;

    // Find index of prediction closest to currentTime
    for (size_t i = 0; i < object.future_predictions.size(); ++i) {
        double time_diff = std::abs(object.future_predictions[i].predicted_time - currentTime);
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
cv::RotatedRect rect1(cv::Point2f(x1, y1), cv::Size2f(len1, wid1), angle1);
cv::RotatedRect rect2(cv::Point2f(x2, y2), cv::Size2f(len2, wid2), angle2);

std::vector<cv::Point2f> intersection;
float intersection_area = (float)cv::rotatedRectangleIntersection(rect1, rect2, intersection) == cv::INTERSECT_FULL ? //? meaning: condition? return if true : return if false;
  cv::contourArea(intersection) : 0.0f;

float area1 = len1 * wid1;
float area2 = len2 * wid2;
float union_area = area1 + area2 - intersection_area;

if (union_area > 0.0f) {
    return intersection_area / union_area;
} else {
    return 0.0f;
}
}


void BoundingBoxNode::pubKfMarkerArrays(std::string frame_id){
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for(auto const& trackedObject : trackedObjectsList){
        visualization_msgs::msg::Marker marker;
        Eigen::VectorXd state = trackedObject.kf.x;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.pose.position.x = state[0];
        marker.pose.position.y = state[1];
        marker.pose.position.z = 0.0; // Z is ignored
        marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), state[6]));
        marker.scale.x = state[5]; //width
        marker.scale.y = state[4]; //length = height
        marker.scale.z = 0.1; // Small height for 2D box
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        marker.id = id++;
        marker.header.frame_id = frame_id;
        marker.lifetime = rclcpp::Duration(0, 500000000); // 0.5s
        marker_array.markers.push_back(marker);


        // Velocity Arrow Marker
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.header.frame_id = frame_id;
        arrow_marker.id = id++; // Unique ID for each marker
        arrow_marker.scale.x = state[5]/8; // Shaft diameter
        arrow_marker.scale.y = state[5]/4;  // Arrowhead diameter
        arrow_marker.scale.z = 0.0;  // Not used for arrows
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 1.0; // Blue arrow
        arrow_marker.color.a = 1.0; // Fully visible

        geometry_msgs::msg::Point start, end;
        start.x = state[0];
        start.y = state[1];
        start.z = 0.1; // Slightly above the box
        end.x = state[0] + state[2]; // velx
        end.y = state[1] + state[3]; // vely
        end.z = 0.1; // Keep in the same plane

        arrow_marker.points.push_back(start);
        arrow_marker.points.push_back(end);
        arrow_marker.lifetime = rclcpp::Duration(0, 500000000); // 0.5s


        marker_array.markers.push_back(arrow_marker);
    }

    kf_bbox_pub_->publish(marker_array);
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
            cout << "updating a kf";
            /*if (trackedObject.rectangle.width>trackedObject.rectangle.height){
                std::swap(trackedObject.rectangle.width,trackedObject.rectangle.height);
                trackedObject.rectangle.angle_width =  trackedObject.rectangle.angle_height;    //trackedObject.rectangle.angle_width-M_PI/2; //need to fix this
                }*/
            z << trackedObject.rectangle.center.x, trackedObject.rectangle.center.y,  trackedObject.rectangle.height, trackedObject.rectangle.width, trackedObject.rectangle.angle_width;
            cout << z<<endl;
            trackedObject.kf.update(z);
        }
    }

}
void BoundingBoxNode::DataAssociate(){
    costMatrix_.clear();
    assignment_.clear(); //HungAlgo.solve already does this, but this way is easier to read the code
    int it=0;
    for(auto const& currentObject : currentObjectsList){
        //https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
        //std::vector<double> vec(trackedObject.costVector.data(), trackedObject.costVector.data() + trackedObject.costVector.size());
        costMatrix_.push_back(currentObject.costVector);
        it++;
        /*// Print costMatrix_
        cout<< "New row in costMatrix_:";
        //std::ostringstream oss;
        for (const auto& value : currentObject.costVector) {
            cout << value;
        }*/
        
    } 
    
    
    /*
    for (auto const& trackedObject : trackedObjectsList) {
        RCLCPP_INFO(this->get_logger(), "Processing object...");
    
        // Check if costVector is initialized
        RCLCPP_INFO(this->get_logger(), "costVector size: %ld", trackedObject.costVector.size());
    
        // Ensure costVector is non-empty before accessing data()
        if (trackedObject.costVector.size() == 0) {
            RCLCPP_WARN(this->get_logger(), "Skipping object with empty costVector!");
            continue;
        }
    
        std::vector<double> vec(trackedObject.costVector.data(), 
                                trackedObject.costVector.data() + trackedObject.costVector.size());
        costMatrix_.push_back(vec);
    
        // Print vector contents
        std::ostringstream oss;
        for (const auto& value : vec) {
            oss << value << " ";
        }
        RCLCPP_INFO(this->get_logger(), "New row: %s", oss.str().c_str());
    
        it++; // (This line is unnecessary if 'it' is not used)
    }
    */
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
    //for(auto it = trackedObjectsList.begin(); it != trackedObjectsList.end();){
        objectTracker& currentObject = currentObjectsList[it];
        int assignedObject= assignment_[it];
        //cout << assignedObject <<", ";
        if (costMatrix_[it][assignedObject]> cost_threshold_){
            assignment_[it]=-1; //this is done this way, because it is helpfull in finMissingNumbers
            assignedObject= assignment_[it];
        }
        //i need to change these lists to vectors or queue, so i can do random access to them https://stackoverflow.com/questions/5733842/how-to-get-a-certain-element-in-a-list-given-the-position
        if (assignedObject>=0 && assignedObject< nCols){
            objectTracker& trackedObject = trackedObjectsList[assignedObject];
            trackedObject.updateStepKF=true;
            trackedObject.rectangle= currentObject.rectangle;   //bouding box rectangle has all the needed information to update KF
            if (trackedObject.newObject) {
                trackedObject.newObjectCounter++;
                if (trackedObject.newObjectCounter >trackedObject.newObjectThreshold){ //contiditon to add object
                    trackedObject.newObject=false;
                }
            trackedObject.ocludedCounter=0; //reset on the oclusion counter
            }
        }
        else {
            trackedObjectsList.push_back(currentObject);//this newly created object already comes with .updateStepKF=true, newObject = true;newObjectCounter = 0;
        }
    }
    
    
   

    //to deal with new objects that were not associated to any old objects:
    std::vector<int> missing = findMissingNumbers(assignment_, nCols);
    cout << "missing matrix:" ;
    for (int num : missing) {
        objectTracker& trackedObject = trackedObjectsList[num];
        // no match for this old tracked object
        trackedObject.updateStepKF=false;
        trackedObject.ocludedCounter++;
        cout << num << " , ";
        if (trackedObject.ocludedCounter>trackedObject.pruneThreshold){ //condition to prune
            //lets not track this object anymore
            trackedObjectsList.erase(trackedObjectsList.begin()+num); //erasing is possible, as the missing indices are sorted in descending order
            cout << endl<< "I HAVE ERASED AN OBJECT NOW" <<endl<<endl;
        }   
        
        }    
    cout << endl;
  
        //trackedObjectsList.push_back(currentObjectsList[num])
}
    
        
std::vector<int> BoundingBoxNode::findMissingNumbers(const std::vector<int>& assignment, int m) {
    std::vector<bool> present(m, false); // Track which numbers are in assignment
    std::vector<int> missing;

    // Mark numbers that appear in assignment
    for (int num : assignment) {
        if (num >= 0 && num < m) {  // Ensure index is within bounds
            present[num] = true;
        } //else {
          //  RCLCPP_ERROR(this->get_logger(), "Invalid value in assignment: %d (out of range 0-%d)", num, m-1);
        //}
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

    
/*
std::vector<int> BoundingBoxNode::findMissingNumbers(const std::vector<int>& assignment, int m) {
    //assignment_ is a vector<int> with n=3(f.e.) entries. each entry will be a different number from 0 to m=4(f.e.)
    std::vector<bool> present(m, false); // Track which numbers are in assignment
    std::vector<int> missing;

    // Mark numbers that appear in assignment
    for (int num : assignment) {
        present[num] = true;
    }

    // Collect numbers that are missing
    for (int i = 0; i <= m; ++i) {
        if (!present[i]) {
            missing.push_back(i);
        }
    }

    return missing;
}
*/
void BoundingBoxNode::defineCosts(objectTracker& object){
    //cout<< "going to define costs for this object"<< endl;

    //object.costVector.resize(0);
    //object.costVector.resize(trackedObjectsList.size()); //resize deletes all entries, thus leaving an empty sparse vector. Resize because nr of tracked objects can chaneg
    object.costVector.clear();
    int it=0;
    
    for(auto const& trackedObject : trackedObjectsList){
        double aux_var= costFuntion(object, trackedObject);
        //cout << "cost: "<< aux_var << endl;
        object.costVector.push_back(aux_var);
        it++;
    }

}

double BoundingBoxNode::costFuntion(const objectTracker& object, const objectTracker& trackedObject){

    return std::sqrt(  std::pow((object.rectangle.center.x - trackedObject.rectangle.center.x),2) + std::pow((object.rectangle.center.y - trackedObject.rectangle.center.y),2)  );
}

void BoundingBoxNode::predictKalmanFilters(float currentTime){
    for(auto& trackedObject : trackedObjectsList){
        trackedObject.kf.predict(currentTime);
    }


}


void BoundingBoxNode::defineHeight(objectTracker& object){
    //we are always working on evolo's 2D frame, meaning that the pointcloud, bouding box/rectangle coordinates and angles all comes defined in that frame
    //this function will rotate the pointcloud -rectangle.angle_width degrees, this will align the pointcloud to one of the sides of the rectangle.
    //this equates to rotating the rectangle to align it with evolo's axis.
    //after this transform, any point that falls within center.x +/- width/2 and center.y +/- height/2 will be within that bouding box
    //with these points, we will take the maximum height 
    double max_height = std::numeric_limits<double>::lowest(); // Initialize to a very low value
    if(has_received_3dcloud_){
        geometry_msgs::msg::TransformStamped cloud_translation, cloud_rotation;
        cloud_translation.transform.translation.x = -object.rectangle.center.x;
        cloud_translation.transform.translation.y = -object.rectangle.center.y;
        cloud_translation.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0);  // Roll, Pitch, Yaw
        cloud_translation.transform.rotation.x = quat.x();
        cloud_translation.transform.rotation.y = quat.y();
        cloud_translation.transform.rotation.z = quat.z();
        cloud_translation.transform.rotation.w = quat.w();
        // Create a quaternion for rotation around Z-axis
        quat.setRPY(0, 0, object.rectangle.angle_width);  // Roll, Pitch, Yaw
        cloud_rotation.transform.translation.x = 0.0;
        cloud_rotation.transform.translation.y = 0.0;
        cloud_rotation.transform.translation.z = 0.0;
        cloud_rotation.transform.rotation.x = quat.x();
        cloud_rotation.transform.rotation.y = quat.y();
        cloud_rotation.transform.rotation.z = quat.z();
        cloud_rotation.transform.rotation.w = quat.w();
        // Step 2: Transform the point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_transformed1, pcl_cloud_transformed2;
        pcl::fromROSMsg(*last_3Dcloud_, pcl_cloud);
        pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed1, cloud_translation); //the order here is very important, it does not work if translation and rot are done all at the same time
        pcl_ros::transformPointCloud(pcl_cloud_transformed1, pcl_cloud_transformed2, cloud_rotation);

        // Step 3: Find the maximum height of points inside the bounding box
        for (const auto& point : pcl_cloud_transformed2) {
            // Check if the point falls within the rotated bounding box
            if ((std::abs(point.x) <= object.rectangle.width / 1.5)&&  //should be divided by 2, 1.5 is used to consider points closer to the center of the boat which are taller
                (std::abs(point.y) <= object.rectangle.height / 1.5)){
                        /*
                object.rectangle.center.x - object.rectangle.width / 2.0) <= point.x &&
                point.x <= (object.rectangle.center.x + object.rectangle.width / 2.0) &&
                (object.rectangle.center.y - object.rectangle.height / 2.0) <= point.y &&
                point.y <= (object.rectangle.center.y + object.rectangle.height / 2.0)) {*/
                // Update max height if the current point's Z is higher
                if (point.z > max_height) {
                    max_height = point.z;
                    //cout << "New max height found"<< max_height<<endl;
                }
            }
        }
    }
    // If no points were found inside, default height to 0
    if (max_height == std::numeric_limits<double>::lowest()) {
        max_height = 0.1;
    }

    

    object.height = max_height;

}

/*
KalmanFilter BoundingBoxNode::init_filter(const Eigen::VectorXd& state, const int currentTime) {
    int num_states = 2, num_sensors = 1;
    
    Eigen::MatrixXd F(num_states, num_states);
    F << 1, 1, 0, 1;  // motion model (constant velocity)

    Eigen::MatrixXd H(num_sensors, num_states);
    H << 1, 0;  // Measuring position only

    KalmanFilter kf(num_states, num_sensors, F, H);

    kf.x=state;
    kf.time=currentTime;

    return kf;
}

*/
void BoundingBoxNode::pointCloud3DBuffer(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    //saves the last received
    last_3Dcloud_=msg;
    has_received_3dcloud_=true;
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
    marker.lifetime = rclcpp::Duration(0, 500000000); // 0.5s
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
    marker.lifetime = rclcpp::Duration(0, 500000000); // 0.5s
    return res;
}