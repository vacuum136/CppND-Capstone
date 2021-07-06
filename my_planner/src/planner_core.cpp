#include <my_planner/planner_core.h>
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/costmap_2d.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseGlobalPlanner)

namespace my_planner {

MyPlanner::MyPlanner() :
  costmap_(nullptr), convert_offset_(0), initialized_{false} {}

MyPlanner::MyPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
  frame_id_(frame_id)
{
  initialize(name, costmap, frame_id);
}
MyPlanner::~MyPlanner() {}

void MyPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void MyPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{
  if(!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    costmap_ = costmap;
    frame_id_ = frame_id;
    initialized_ = true;
    // if this param is true, the waypoints is located at the center of the grid
    private_nh.param("path_at_grid_center", path_at_grid_center_, true);
    if(path_at_grid_center_) {
      // 
      convert_offset_ = 0.5;
    }

  } else {
    ROS_WARN("This planner has already been initialized!");
  }
}

bool MyPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                         std::vector<geometry_msgs::PoseStamped>& plan)
{
}


// World is divided into grids of size cells_x * cells_y
// cells_x equals length[meter] of x divided by resolution[meters/cell], as same in y.
// Loss happens during the conversion, all 
bool MyPlanner::worldToMap(double x, double y)
{
  double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();
  if(x < origin_x || y < origin_y) {
    return false;
  }
  x = (x - origin_x) / resolution;
  y = (y - origin_y) / resolution;
  if (x < costmap_->getSizeInCellsX() && y < costmap_->getSizeInCellsY()) {
    return true;
  }
  return false;
}

void MyPlanner::mapToWorld(double x, double y)
{
  x = costmap_->getOriginX() + (x + convert_offset_) * costmap_->getResolution();
  y = costmap_->getOriginY() + (y + convert_offset_) * costmap_->getResolution();
}


} // end namespace my_planner