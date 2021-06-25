#include <ros/ros.h>
#include "robot_simulation/path_planner.h"

PathPlanner::PathPlanner
  (int start, int goal, std::vector<int> &costmap, 
   int width, int height, float resolution, ExpandVisual *ev) : 
   start_index_(start), goal_index_(goal), costmap_(costmap), 
  costmap_width_(width), costmap_height_(height), resolution_(resolution), expandviz_(ev){}

bool PathPlanner::AStarSearch(){
  // open list to store the index to be checked
  std::vector<Index> open_list{};
  // store the index checked
  std::set<int> close_list{};
  // map between index and it's parents
  std::unordered_map<int, int> parents{};
  
  open_list.emplace_back(start_index_, 0.f);

  ROS_INFO("Path planner: initialization done");

  // loop as long as there is index in open list
  while(!open_list.empty()){
    // get the index with lowest cost 
    std::sort(open_list.begin(), open_list.end(),[](const Index &a, const Index &b){return (a.g_cost+a.h_cost) > (b.g_cost+b.h_cost);});
    Index current = open_list.back();
    open_list.pop_back();
    // marked this index checked
    close_list.insert(current.i);
    expandviz_->setColor(current.i, "pale_yellow");
    if(current.i == goal_index_){
      ConstructFinalPath(current.i, parents);
      break;
    }  
    
    std::vector<Index> neighbors = findNeighbors(current.i);

    for(Index &neighbor : neighbors){
      if(close_list.count(neighbor.i)) continue;
      // calculate new g_cost, neighbor'cost is only step cost for now
      float g_cost = current.g_cost + neighbor.g_cost;
      float h_cost = CalculateHValue(neighbor.i);

      auto iter = std::find(open_list.begin(), open_list.end(), neighbor);
      if(iter != open_list.end()){ // neighbor already in open list
        if(g_cost < iter->g_cost){
          // update the cost value of the index in open list
          iter->g_cost = g_cost;
          parents[iter->i] = current.i;
        }
      } else {
        neighbor.g_cost = g_cost;
        neighbor.h_cost = h_cost;
        parents[neighbor.i] = current.i;
        open_list.push_back(neighbor);
        expandviz_->setColor(neighbor.i,"orange");
      }
    }
    sleep(1);
  }
  return !path_.empty();
}

void PathPlanner::ConstructFinalPath(int current, std::unordered_map<int,int> &parents) {
  // Create path_found vector
  do
  {
    path_.push_back(current);
    current = parents[current];
  } while (current != start_index_);

  path_.push_back(start_index_);

  //reverse the founded path
  std::reverse(path_.begin(),path_.end());
}

// Calculate H value
float PathPlanner::CalculateHValue(int index) {
	float x = index % costmap_width_, y = index / costmap_width_;
  float goal_x = goal_index_ % costmap_width_, goal_y = goal_index_ / costmap_width_;
  return (abs(goal_x - x) + abs(goal_y - y)) * resolution_; // step cost = resolution 
}

// find neighbors of current index
std::vector<Index> PathPlanner::findNeighbors(int index){
  std::vector<Index> neighbor{};
  float orthonogal_step_cost = resolution_;
  float diagonal_step_cost = orthonogal_step_cost * 1.41421;
  float lethal_cost = 1;

  int upper = index - costmap_width_;
  if(upper>0){
    if(costmap_[upper] < lethal_cost){
      float step_cost = orthonogal_step_cost + costmap_[upper]/255;
      neighbor.emplace_back(upper,step_cost);
    }
  }
  int left = index - 1;
  if(left % costmap_width_ > 0){
    if(costmap_[left] < lethal_cost){
      float step_cost = orthonogal_step_cost + costmap_[left]/255;
      neighbor.emplace_back(left,step_cost);
    }
  }
  int upper_left = index - costmap_width_ - 1;
  if(upper_left > 0 && upper_left % costmap_width_ > 0){
    if(costmap_[upper_left] < lethal_cost){
      float step_cost = diagonal_step_cost + costmap_[upper_left]/255;
      neighbor.emplace_back(upper_left,step_cost);
    }
  }
  int upper_right = index - costmap_width_ + 1;
  if(upper_right > 0 && upper_right % costmap_width_ != (costmap_width_ - 1)){
    if(costmap_[upper_right] < lethal_cost){
      float step_cost = diagonal_step_cost + costmap_[upper_right]/255;
      neighbor.emplace_back(upper_right,step_cost);
    }
  }
  int right = index + 1;
  if(right % costmap_width_ != (costmap_width_ - 1)){
    if(costmap_[right] < lethal_cost){
      float step_cost = orthonogal_step_cost + costmap_[right]/255;
      neighbor.emplace_back(right,step_cost);
    }
  }
  int lower_left = index + costmap_width_ - 1;
  if(lower_left < costmap_height_ * costmap_width_ && lower_left % costmap_width_ != 0){
    if(costmap_[lower_left] < lethal_cost){
      float step_cost = diagonal_step_cost + costmap_[lower_left]/255;
      neighbor.emplace_back(lower_left,step_cost);
    }
  }
  int lower = index + costmap_width_;
  if(lower <= costmap_height_ * costmap_width_){
    if(costmap_[lower] < lethal_cost){
      float step_cost = orthonogal_step_cost + costmap_[lower]/255;
      neighbor.emplace_back(lower,step_cost);
    }
  }
  int lower_right = index + costmap_width_ + 1;
  if(lower_right <= costmap_height_ * costmap_width_ && lower_right % costmap_width_ != (costmap_width_ - 1)){
    if(costmap_[lower_right] < lethal_cost){
      float step_cost = diagonal_step_cost + costmap_[lower_right]/255;
      neighbor.emplace_back(lower_right,step_cost);
    }
  }

  return neighbor;
}


