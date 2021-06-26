#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <vector>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <robot_simulation/expand_visual.h>

/**
 * @class Index
 * @brief Class stores index with its g and h cost value.
 */
class Index {
  public:
    Index(int a, float b):i(a), g_cost(b), h_cost(0){};
    int i;
    float g_cost;
    float h_cost;
    bool operator== (const Index& other){ return i == other.i; }
};
/**
 * @class PathPlanner
 * @brief a global path planner for robot
 */
class PathPlanner {
public:
  // PathPlanner constructor
  PathPlanner();
  /*
  * @brief PathPlanner constructor
  * @param start index of the start position converted into the grid
  * @param goal index of the goal position converted into the grid
  * @param costmap reference to the costmap
  * @param width width of the grid (= number of cells each row)
  * @param height height of the grid (= number of rows)
  * @param resolution resolution of the costmap [meter/grid]
  * @param *ev pointer to the ExpandVisual object
  */
  PathPlanner(int start, int goal, std::vector<int> &costmap, 
              int width, int height, float resolution, ExpandVisual *ev);
  /*
  * @brief find the neighbors of one cell
  * @param index index of the cell in the grid
  */
  std::vector<Index> findNeighbors(int index);
  /*
  * @brief calculate the heuristic value of one cell, here use manhattan distance
  * @param index index of the cell in the grid
  */  
  float CalculateHValue(int index);
  /*
  * @brief expand the cells using A* algorithm
  * @return true if path founded and false if failed  
  */  
  bool AStarSearch();
  /*
  * @brief connecte all the waypoints founded by A* algorithm
  * @param index
  * @param parents a hash table mapping one cell's index and its parents'
  */ 
  void ConstructFinalPath(int index, std::unordered_map<int,int> &parents);
  /*
  * @brief get the finally founded path
  */   
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