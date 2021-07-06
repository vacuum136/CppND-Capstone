#ifndef PLANNER_CORE_H_
#define PLANNER_CORE_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>


namespace my_planner {

class MyPlanner : public nav_core::BaseGlobalPlanner
{
public:
  MyPlanner();
  MyPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);
  ~MyPlanner();
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  void initialize(std::string name, costmap_2d::Costmap2D costmap, std::string frame_id);
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

private:
  void mapToWorld(double x, double y);
  void worldToMap(double x, double y);
  costmap_2d::Costmap2D* costmap_;
  double convert_offset_;
  bool path_at_grid_center_;
  std::string frame_id_;
  bool initialized_;
};

} // end namespace my_planner
#endif