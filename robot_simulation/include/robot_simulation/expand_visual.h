#ifndef EXPAND_VISUAL_H_
#define EXPAND_VISUAL_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <unordered_map>

struct Point
{
  float x, y, z;
};

/**
 * @class ExpandVisual
 * @brief Use Point Cloud as visualier of the expanding process of path planning algorithm
 */
class ExpandVisual{
public:
  // ExpandVisual Constructor
  ExpandVisual();
  /**
   * @brief  Constructor for the ExpandVisual
   * @param  costmap reference to the costmap
   * @param  resolution resolution of the costmap [meter/grid]
   * @param  origin the coordinates of the cell[0] of the grid with reference to world
   * @param  start index of the start position converted into the grid
   * @param  goal index of the goal position converted into the grid
   * @param  width width of the grid (= number of grid each row)
   */
  ExpandVisual(std::vector<int> &costmap, float resolution, Point orgin, int start, int goal, int width);
    /**
   * @brief  initial a point cloud using all points
   */
  void init_points();
  /**
   * @brief  set the color of the current point and publish pointcloud msg
   * @param  index index of the current point
   * @param  color color of the point in cloud point
   */
  void setColor(int index, std::string color);
  /**
   * @brief  convert a point index into its [x,y,z] of the world
   */
  Point indexToWorld(int index);

private:
  std::vector<int> &costmap_;
  float resolution_;
  Point origin_;
  int start_;
  int goal_;
  int width_;
  int id{0};
  std::unordered_map<std::string,int> color_{};
  std::string frame_{"map"};
  ros::Publisher pub_;
  ros::NodeHandle nh_;
  pcl::PointCloud<pcl::PointXYZRGBA> cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr msg_;
};

#endif