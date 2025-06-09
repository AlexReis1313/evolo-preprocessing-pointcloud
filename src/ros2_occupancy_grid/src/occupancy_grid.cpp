#include "occupancy_grid/occupancy_grid.h"

#include <assert.h>
#include <iostream>
#include <cmath>

OccupancyGrid::OccupancyGrid(unsigned int grid_size, double cell_size)
    : grid_size_{grid_size}, cell_size_{cell_size}
{
  std::cout << "makingOccuGrid" << std::endl;

  num_cells_ = floor(grid_size_ / cell_size_);
  grid_center_ = {num_cells_ / 2, num_cells_ / 2};
  map_.resize(num_cells_, num_cells_);
  map_.setConstant(-1.0);
}

void OccupancyGrid::toRosMsg(nav_msgs::msg::OccupancyGrid& occupancy_grid_msg,tf2::Transform & robot_pose_inOCGMapFrame)
{
  occupancy_grid_msg.info.width = num_cells_;
  occupancy_grid_msg.info.height = num_cells_;
  occupancy_grid_msg.info.resolution = cell_size_;

  tf2::Vector3 map_offset = robot_pose_inOCGMapFrame.getOrigin();
  occupancy_grid_msg.info.origin.position.x = -grid_center_.x * cell_size_ - map_offset.x();
  occupancy_grid_msg.info.origin.position.y = -grid_center_.y * cell_size_ - map_offset.y();
  double roll, pitch, yaw;
  tf2::Matrix3x3(robot_pose_inOCGMapFrame.getRotation()).getRPY(roll, pitch, yaw);
  tf2::Quaternion corrected_rotation;
  corrected_rotation.setRPY(0, 0, yaw);  // Invert yaw to compensate robot rotation
  occupancy_grid_msg.info.origin.orientation = tf2::toMsg(corrected_rotation);


  occupancy_grid_msg.header.frame_id = "odom";

  const int num_cells_grid = num_cells_ * num_cells_;
  for (size_t i = 0; i < num_cells_grid; i++) {
    double& occ_prob = map_.data()[i];
    if (occ_prob == -1.0) {
      occupancy_grid_msg.data.push_back(-1);
    }
    else {
      occupancy_grid_msg.data.push_back(occ_prob * 100);
    }
  }
}

void OccupancyGrid::update(double delta_x, double delta_y, double delta_yaw)
{
 
  Eigen::MatrixXd temp_map(num_cells_, num_cells_);
  temp_map.setConstant(-1.0);
  // Convert position delta to cell delta
  //delta_x,  delta_y and delta_yaw are in relation to odom frame. not to map frame. delta_x_grid is then not given by delta_x / cell_size
  //i must use last yaw or something to convert that
  double delta_x_grid = -delta_x / cell_size_;
  double delta_y_grid = -delta_y / cell_size_;
  double cos_dyaw = cos(-delta_yaw);
  double sin_dyaw = sin(-delta_yaw);
  

  std::cout << "x: "<< delta_x <<" y: " << delta_y << " yaw :" << delta_yaw << std::endl;//"grid_center_.x,y "<< grid_center_.x << grid_center_.y << std::endl;
  Eigen::MatrixXd transformation_matrix(3, 3);
  transformation_matrix << cos_dyaw, -sin_dyaw, delta_x_grid,
                           sin_dyaw, cos_dyaw, delta_y_grid, 
                           0.0,     0.0, 1.0;
  for (int x = 0; x < num_cells_; ++x) {
    for (int y = 0; y < num_cells_; ++y) {
      Eigen::VectorXd old_indices(3);
      // Translate (x, y) to origin
      old_indices << x - grid_center_.x, y - grid_center_.y, 1.0;
      // Transform according to robot movement
      Eigen::VectorXd new_indices = transformation_matrix * old_indices;
      // Translate back to map indices
      new_indices(0) += grid_center_.x;
      new_indices(1) += grid_center_.y;
      if (isInGridBounds(static_cast<int>(round(new_indices(0))), static_cast<int>(round(new_indices(1))))) { //floor rounds down - needed because if a point is at 0.8, 0.9, it belongs to the cell 0,0 in a 1mx1m grid map
        temp_map(static_cast<int>(round(new_indices(0))), static_cast<int>(round(new_indices(1)))) = map_(x, y);
      }
    }
  } /*
  std::cout << "x: "<< delta_x <<" y: " << delta_y << " yaw :" << delta_yaw << std::endl;//"grid_center_.x,y "<< grid_center_.x << grid_center_.y << std::endl;
  Eigen::MatrixXd transformation_matrix(3, 3);
  transformation_matrix << cos_dyaw, -sin_dyaw, delta_x,
                           sin_dyaw, cos_dyaw, delta_y, 
                           0.0,     0.0, 1.0;
  for (int x = 0; x < num_cells_; ++x) {
    for (int y = 0; y < num_cells_; ++y) {
      Eigen::VectorXd old_indices(3);
      // Translate (x, y) to origin
      old_indices << (x - grid_center_.x)*cell_size_, (y - grid_center_.y)*cell_size_, 1.0;
      // Transform according to robot movement
      Eigen::VectorXd new_indices = transformation_matrix * old_indices;
      // Translate back to map indices
      new_indices(0) = (new_indices(0)/cell_size_ ) + grid_center_.x;
      new_indices(1) = (new_indices(1)/cell_size_ ) +grid_center_.y;
      if (isInGridBounds(static_cast<int>(floor(new_indices(0))), static_cast<int>(floor(new_indices(1))))) { //floor rounds down - needed because if a point is at 0.8, 0.9, it belongs to the cell 0,0 in a 1mx1m grid map
        temp_map(static_cast<int>(floor(new_indices(0))), static_cast<int>(floor(new_indices(1)))) = map_(x, y);
      }
    }
  }*/
  map_ = temp_map;
}

void OccupancyGrid::update(const std::vector<Point2d<double>>& laser_scan, tf2::Transform & robot_pose_inOCGMapFrame)
{

  // Create vector of free cells and reserve approximate amount of memory for max possible distance
  std::vector<Point2d<double>> transformed_scan;
  for (const auto& pt : laser_scan) {
    tf2::Vector3 transformed = robot_pose_inOCGMapFrame * tf2::Vector3(pt.x, pt.y, 0.0);
    transformed_scan.push_back({transformed.x(), transformed.y()});
  }

  std::vector<Point2d<int>> free_cells;
  free_cells.reserve(floor(grid_size_));
  for (const Point2d<double>& point : transformed_scan) {
    // Convert position of detection to cell indices
    Point2d<int> grid_point{floor(point.x / cell_size_) + grid_center_.x,
                            floor(point.y / cell_size_) + grid_center_.y};
    updateCellProbability(grid_point, CellState::OCCUPIED);
    // Run Bresenham algorithm to get all free cells
    getFreeCells(grid_point, free_cells, robot_pose_inOCGMapFrame);
    for (const Point2d<int>& free_cell : free_cells) {
      updateCellProbability(free_cell, CellState::FREE);
    }
    free_cells.clear();
  }
}

void OccupancyGrid::updateCellProbability(const Point2d<int>& point, CellState state)
{

  // --- Crucial addition: Check bounds before any map access ---
  if (!isInGridBounds(point.x, point.y)) {
    // Log a warning or error, but do NOT crash the program.
    // Use a logger instead of std::cout for ROS 2 nodes in production.
    //std::cout <<  "Attempted to update cell " <<point.x<< point.y <<"which is out of bounds for grid size" << num_cells_<< "Skipping update."<<std::endl;
    return; // Exit the function to prevent out-of-bounds access
  }
  // Calculate new log odds and add to current log odds
  if(map_(point.x, point.y)==-1.0){
    map_(point.x, point.y)=0.5;
  }
  double log_prob{0.0};
  switch (state) {
    case CellState::FREE:
      log_prob = log(p_free_ / (1.0 - p_free_));
      break;
    case CellState::OCCUPIED:
      log_prob = log(p_occ_ / (1.0 - p_occ_));
      break;
    default:
      log_prob = log(p_prior_ / (1.0 - p_prior_));
      break;
  }
  double current_log_prob = log(map_(point.x, point.y) / (1.0 - map_(point.x, point.y)));
  current_log_prob += log_prob;
  // Convert log odds to probability and update cell
  map_(point.x, point.y) = 1.0 - 1.0 / (1 + exp(current_log_prob));
}


void OccupancyGrid::getFreeCells(const Point2d<int>& detection,
                                 std::vector<Point2d<int>>& free_cells, tf2::Transform & robot_pose_inOCGMapFrame)
{ //Bresenham line algorithm

    //int x0 = grid_center_.x;
    //int y0 = grid_center_.y;
    tf2::Vector3 map_offset = robot_pose_inOCGMapFrame.getOrigin();
    int x0 = static_cast<int>(std::floor(map_offset.x() / cell_size_)) + grid_center_.x;
    int y0 = static_cast<int>(std::floor(map_offset.y() / cell_size_)) + grid_center_.y;

    int x1 = detection.x;
    int y1 = detection.y;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1; // if detection is negative point, we move the line in the negative x diretions sx=-1, else we move it to the right sx=1
    int sy = (y0 < y1) ? 1 : -1; //same aplies for y
    int err = dx - dy; 

    while (true) {
        if (isInGridBounds(x0, y0)) {
            free_cells.push_back({x0, y0});
        }

        //if (x0 == x1 && y0 == y1) //stop when we get to the detection point
        //    break;
        if ((x0 - x1) * sx >= 0 && (y0 - y1) * sy >= 0) //improving robustness of stoping criteria
          break;
        int e2 = 2 * err; //variable e2 decides if we need to move in x, in y or both
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

bool OccupancyGrid::isInGridBounds(int x, int y)
{

  if (!(x >= 0 && x < num_cells_)) return false;
  if (!(y >= 0 && y < num_cells_)) return false;
  return true;
}
