#ifndef EXPAND_VISUAL_H_
#define EXPAND_VISUAL_H_

#include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/PointField.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <unordered_map>

struct Point
{
  float x, y, z;
};


class ExpandVisual{
public:
  ExpandVisual();
  ExpandVisual(std::vector<int> &costmap, float resolution, Point orgin, int start, int goal, int width);
  void init_points();
  void setColor(int index, std::string color);
  Point indexToWorld(int index);

private:
  std::vector<int> &costmap_;
  float resolution_;
  Point origin_;
  int start_;
  int goal_;
  int width_;
  int id{0};
  //std::vector<std::pair<Point,int>> points_;
  std::unordered_map<std::string,int> color_{};
  std::string frame_{"map"};
  ros::Publisher pub_;
  ros::NodeHandle nh_;
  // sensor_msgs::PointField field_x;
  // sensor_msgs::PointField field_y;
  // sensor_msgs::PointField field_z;
  // sensor_msgs::PointField field_rgba;
  pcl::PointCloud<pcl::PointXYZRGBA> cloud_;
};

#endif