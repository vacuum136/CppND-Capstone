#include <ros/ros.h>
#include <pp_msgs/PathPlanningPlugin.h>
#include <geometry_msgs/Twist.h>
#include <robot_simulation/path_planner.h>

// callback function of the path planning service whichi processes the request from clients and response with path if found
bool makePlan(pp_msgs::PathPlanningPlugin::Request &req, pp_msgs::PathPlanningPlugin::Response &res)
{
  float resolution = 0.2;
  Point origin{-7.4, -7.4, 0};
  PathPlanner path_planer = PathPlanner(req.start, req.goal, req.costmap_ros, req.width, req.height, resolution, origin);

  ros::Time start_time = ros::Time::now();

  // calculate the path using astar
  if(path_planer.AStarSearch()){
    ROS_INFO("Path planner: Path Founded!");
    res.plan = path_planer.getPath();
  }else{
    ROS_INFO("Path planner: Failed to find the Path!");
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning_service_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService<>("/move_base/SrvClientPlugin/make_plan", makePlan);
  ROS_INFO("Path planning server: Path Planning is ready");
  ros::spin();
  
  return 0;
}