#include "bounding_box_node.hpp"


// Define the global matrices
Eigen::MatrixXd motion_model;
Eigen::MatrixXd measurement_model;

void initMatrices() {

    // Resize the matrices before using <<
    motion_model = Eigen::MatrixXd(num_states, num_states);
    measurement_model = Eigen::MatrixXd(num_sensors, num_states);

    // Initialize the matrices
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
    RCLCPP_INFO(
        this->get_logger(),
        "Started pointcloud subscriber");
}


void BoundingBoxNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    
    float currentTime=rclcpp::Time(msg->header.stamp).seconds();
    //predict next pose on kalman filters
    predictKalmanFilters(currentTime);
    
    // Separate clusters by intensity value
    std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    for (const auto& point : cloud->points) {
        int cluster_id = static_cast<int>(point.intensity);
        if (clusters.find(cluster_id) == clusters.end()) {
            clusters[cluster_id] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        }
        clusters[cluster_id]->points.push_back(point);
    }

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
        marker.header.frame_id = msg->header.frame_id ;
        marker_array.markers.push_back(marker);
        currentObjectsList.push_back(object);
    }



    DataAssociate();    //HUngarian algorithm for matching - 


    updateKalmanFilters();

    pubKfMarkerArrays(msg->header.frame_id );


    // Publish the bounding boxes
    bbox_pub_->publish(marker_array);

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
        marker_array.markers.push_back(marker);


        // Velocity Arrow Marker
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.header.frame_id = frame_id;
        arrow_marker.id = id++; // Unique ID for each marker
        arrow_marker.scale.x = state[5]/4; // Shaft diameter
        arrow_marker.scale.y = state[5]/2;  // Arrowhead diameter
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

        marker_array.markers.push_back(arrow_marker);
    }

    kf_bbox_pub_->publish(marker_array);



}
void BoundingBoxNode::updateKalmanFilters(){
    Eigen::VectorXd z(num_sensors);
    for(auto& trackedObject : trackedObjectsList){
        if (trackedObject.updateStepKF){
            z << trackedObject.rectangle.center.x, trackedObject.rectangle.center.y,  trackedObject.rectangle.height, trackedObject.rectangle.width, trackedObject.rectangle.angle_width;
            trackedObject.kf.update(z);
        }
    }

}
void BoundingBoxNode::DataAssociate(){
    costMatrix_.clear();
    assignment_.clear(); //HungAlgo.solve already does this, but this way is easier to read the code
    int it=0;
    for(auto const& trackedObject : trackedObjectsList){
        //https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
        std::vector<double> vec(trackedObject.costVector.data(), trackedObject.costVector.data() + trackedObject.costVector.size());
        costMatrix_.push_back(vec);
        it++;
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

    double cost = HungAlgo.Solve(costMatrix_, assignment_);
    unsigned int nRows = costMatrix_.size();
	unsigned int nCols = costMatrix_[0].size();


    it=0;
    for(auto it = trackedObjectsList.begin(); it != trackedObjectsList.end();){
        objectTracker& trackedObject = *it;
        int assignedObject= assignment_[std::distance(trackedObjectsList.begin(), it)];
        objectTracker& currentObject = std::next(currentObjectsList.begin(), assignedObject);

        if (assignedObject>=0 && assignedObject< nRows ){
            trackedObject.updateStepKF=true;
            trackedObject.rectangle= currentObject.rectangle;   //bouding box rectangle has all the needed information to update KF
            if (trackedObject.newObject) {
                trackedObject.newObjectCounter++;
                if (trackedObject.newObjectCounter >trackedObject.newObjectThreshold){ //contiditon to add object
                    trackedObject.newObject=false;
                }
            trackedObject.ocludedCounter=0; //reset on the oclusion couter
            }
        } else {
            // no match for this old tracked object
            trackedObject.updateStepKF=false;
            trackedObject.ocludedCounter++;
        }
        if (trackedObject.ocludedCounter>trackedObject.pruneThreshold){ //condition to prune
                //lets not track this object anymore
                trackedObjectsList.erase(it);

        }else{it++;}
    }
    //to deal with new objects that were not associated to any old objects:
    std::vector<int> missing = findMissingNumbers(assignment_, nCols);
    for (int num : missing) {
        auto it = std::next(currentObjectsList.begin(), num);
        trackedObjectsList.push_back(*it);
        //trackedObjectsList.push_back(currentObjectsList[num])//this newly created object already comes with .updateStepKF=true, newObject = true;newObjectCounter = 0;
    }    
}
    
        

    

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
void BoundingBoxNode::defineCosts(objectTracker& object){
    object.costVector.resize(trackedObjectsList.size()); //resize deletes all entries, thus leaving an empty sparse vector. Resize because nr of tracked objects can chaneg
    int it=0;
    for(auto const& trackedObject : trackedObjectsList){
        object.costVector(it)=costFuntion(object, trackedObject);
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
    //marker.lifetime = rclcpp::Duration(0, 500000000); // 0.5s
    return res;
}