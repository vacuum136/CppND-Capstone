#include <robot_simulation/expand_visual.h>

ExpandVisual::ExpandVisual
  (std::vector<int> &costmap, float resolution, Point origin, int start, int goal, int width):
  costmap_(costmap), resolution_(resolution), origin_(origin), start_(start), goal_(goal), width_(width){
    //initial the hash table [color name, #ARGB]
    color_ = {{"green",4278255360}, //#FF00FF00
              {"red",4294901760},   //#FFFF0000
              {"blue",4278190335},  //#FF0000FF
              {"pale_yellow",4293918464},
              {"lime_green",4284802916},
              {"orange",4294944000}};
    pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>("/expand_visual", 1000);
    init_points();
  }


void ExpandVisual::init_points(){
  for(int idx = 0; idx < costmap_.size(); ++idx){
    Point p = indexToWorld(idx);
    pcl::PointXYZRGBA newpoint;
    newpoint.x = p.x;
    newpoint.y = p.y;
    newpoint.z = p.z;
    newpoint.rgba = 0;
    cloud_.points.push_back(newpoint);
  }
  cloud_.points[start_].z = 0.1;
  cloud_.points[start_].rgba = color_["blue"];
  cloud_.points[goal_].z = 0.1;
  cloud_.points[goal_].rgba = color_["red"];
  msg_ = cloud_.makeShared();
  msg_->header.frame_id = frame_;
}

void ExpandVisual::setColor(int index, std::string color){
  if(index != start_ && index != goal_)
    msg_->points[index].rgba = color_[color];
  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr msg = cloud_.makeShared();
  pcl_conversions::toPCL(ros::Time::now(),msg_->header.stamp);
  pub_.publish(msg_);
}

Point ExpandVisual::indexToWorld(int index){
  float grid_x = index % width_;
  float grid_y = index / width_;
  float x = resolution_ * grid_x + origin_.x + resolution_ / 2;
  float y = resolution_ * grid_y + origin_.y + resolution_ / 2;
  float z = 0;
  return Point{x,y,z};
}