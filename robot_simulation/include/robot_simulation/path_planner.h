#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <vector>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <robot_simulation/expand_visual.h>

class Index {
  public:
    Index(int a, float b):i(a), g_cost(b), h_cost(0){};
    int i;
    float g_cost;
    float h_cost;
    bool operator== (const Index& other){ return i == other.i; }
};

class PathPlanner {
public:
  // PathPlanner constructor
  PathPlanner();

  // PathPlanner constructor
  PathPlanner(int start, int goal, std::vector<int> &costmap, 
              int width, int height, float resolution, ExpandVisual *ev);
  
  std::vector<Index> findNeighbors(int index);
  float CalculateHValue(int index);
  bool AStarSearch();
  void ConstructFinalPath(int index, std::unordered_map<int,int> &parents);
  std::vector<int> getPath(){return path_;}
private:
  // cost map params
  std::vector<int> &costmap_;
  float resolution_;
  int costmap_width_;
  int costmap_height_;
  // Point origin_; // map's origin with reference to world 
  // start and goal
  int start_index_;
  int goal_index_;
  // founded path
  std::vector<int> path_;
  ExpandVisual *expandviz_;
};

#endif