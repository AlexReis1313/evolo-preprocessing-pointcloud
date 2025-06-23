#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp> 

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unordered_set>


/*
template <typename T>
struct Point2d {
  T x;
  T y;
};*/
template<typename T>
struct Point2d {
  T x;
  T y;

  // Equality operator — needed for unordered_set to compare elements
  bool operator==(const Point2d<T>& other) const {
    return x == other.x && y == other.y;
  }
};

#include <functional>  // for std::hash
namespace std {
  template <>
  struct hash<Point2d<int>> {
    size_t operator()(const Point2d<int>& pt) const noexcept {
      size_t hx = std::hash<int>()(pt.x);
      size_t hy = std::hash<int>()(pt.y);
      return hx ^ (hy << 1); //
    }
  };
}

enum class CellState : unsigned int { FREE, OCCUPIED };

class OccupancyGrid {
 public:
  OccupancyGrid(unsigned int grid_size, double cell_size);

  void toRosMsg(nav_msgs::msg::OccupancyGrid& occupancy_grid_msg,tf2::Transform & robot_pose_inOCGMapFrame);
  void update(double delta_x, double delta_y, double delta_yaw);
  void update(Eigen::MatrixXd& local_map, double delta_x, double delta_y, double delta_yaw);
  void update(const std::vector<Point2d<double>>& laser_scan, tf2::Transform & robot_pose_inOCGMapFrame, bool & bayesFilterType);
  Eigen::MatrixXd getMap();
  bool checkOccupancy(const Point2d<double>& point,tf2::Transform & robot_pose_inOCGMapFrame);
  void fillFreeBetweenOccupied();


 private:
  void updateCellProbability(const Point2d<int>& point, CellState state,   bool & bayesFilterType);
  void getFreeCells(const Point2d<int>& detection, std::vector<Point2d<int>>& free_cells,int& x0_, int & y0_);
  bool isInGridBounds(int x, int y);
  Eigen::MatrixXd map_;
  unsigned int grid_size_{20};
  double cell_size_{0.1};
  unsigned int num_cells_{200};
  Point2d<int> grid_center_{100, 100};
  const double p_free_{0.3};
  const double p_occ_{0.99};
  const double p_prior_{0.5};
  
};

#endif  // OCCUPANCYGRID_H