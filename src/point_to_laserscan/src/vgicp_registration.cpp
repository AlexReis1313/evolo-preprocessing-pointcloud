// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT

#include  "pointcloud_to_laserscan/vgicp_registration.hpp"
#include <iostream>

using namespace small_gicp;
using namespace tf2;
#include <tf2_eigen/tf2_eigen.hpp>

// Constructor#include "pointcloud_to_laserscan/point_cloud_registration.hpp"
#include <iostream>

// Constructor
vgicpRegistrationClass::vgicpRegistrationClass() : firstTime_(true) {
    reg_.setNumThreads(4);
    reg_.setCorrespondenceRandomness(20);
    reg_.setMaxCorrespondenceDistance(1.0);
    reg_.setVoxelResolution(1.0);
    reg_.setRegistrationType("VGICP"); 
    last_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    new_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    new_cloud_transformed_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

void vgicpRegistrationClass::setLastCloud(const pcl::PointCloud<pcl::PointXYZ>& last_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_last_cloud = voxelgrid_sampling_omp(last_cloud, 0.25);

    last_cloud_ = downsampled_last_cloud;
    firstTime_ = false;
    reg_.setInputSource(downsampled_last_cloud);
}

void vgicpRegistrationClass::setNewCloud(const pcl::PointCloud<pcl::PointXYZ>& new_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_new_cloud = voxelgrid_sampling_omp(new_cloud, 0.25);

    new_cloud_ = downsampled_new_cloud;
    firstTime_ = false;
    reg_.setInputTarget(downsampled_new_cloud);
}

pcl::PointCloud<pcl::PointXYZ> vgicpRegistrationClass::getNewTransformedCloud() const {
    if (!firstTime_) {
        pcl_ros::transformPointCloud(*new_cloud_, *new_cloud_transformed_, ICP_output_transform_);
    }
    return *new_cloud_transformed_;
}

const geometry_msgs::msg::TransformStamped& vgicpRegistrationClass::getTransformation4NewCloud() const {
    return ICP_output_transform_;
}

void vgicpRegistrationClass::computeRegistration() {
    if (!firstTime_) {
        new_cloud_transformed_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        reg_.align(*new_cloud_transformed_);
        Eigen::Matrix4f transform_matrix = reg_.getFinalTransformation();

        Eigen::Affine3d affine_transform;
        affine_transform.matrix() = transform_matrix.cast<double>();

        // Convert Eigen::Affine3d to ROS transform
        ICP_output_transform_ = tf2::eigenToTransform(affine_transform);
    }
}

void vgicpRegistrationClass::swapNewLastCloud() {
    std::swap(last_cloud_, new_cloud_);
    reg_.swapSourceAndTarget();
}
