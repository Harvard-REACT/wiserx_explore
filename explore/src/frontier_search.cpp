#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <explore/costmap_tools.h>

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size, double range,
                               float utility_alpha_parameter, float utility_beta_parameter)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , sensor_range_(range)
  , alpha_parameter_(utility_alpha_parameter)
  , beta_parameter_(utility_beta_parameter)
{
  //Calculate max size permissible for any frontier. Frontiers larger than this will be broken down
  max_frontier_size_ = sensor_range_ ;
}

/**
 * @brief Original frontier search function
 * @ Deprecated
 * */
std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
{
  std::vector<Frontier> frontier_list;
  float f_cost_min = 100000, f_cost_max = 0;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // ROS_INFO("################## Map size = %d, %d ########################", size_x_, size_y_);

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);

  // ROS_INFO("World pos: %f, %f", position.x, position.y);
  // ROS_INFO("Map pos: %d, %d", mx, my);
  // ROS_INFO("pos index= %d", pos);

  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  }  
  else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }
  
  int uexp_cell_count = 0, information_gain=0;
  unsigned fmx, fmy;

  // set travel and information gain costs of frontiers
  for (auto& frontier : frontier_list) 
  {
    frontier.cost = frontierCost(frontier); //Default cost function
  }

  // Sort the total calculated cost
  std::sort(
    frontier_list.begin(), frontier_list.end(),
    [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

/**
 * @brief Generate utility of the frontiers
 * */
std::vector<Frontier> FrontierSearch::searchFrontiers(geometry_msgs::Point& position,
                                                      unsigned int x_env_map_max_limit, 
                                                      unsigned int y_env_map_max_limit,
                                                      unsigned int x_env_map_min_limit,
                                                      unsigned int y_env_map_min_limit)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) 
  {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();


  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);

  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  /* Find frontiers */
  while (!bfs.empty()) 
  {
    unsigned int idx = bfs.front();
    bfs.pop();

    // // FOR HARDWARE EXPERIEMNETS if environment does not match the map size
    unsigned int idx_coord_x, idx_coord_y;
    costmap_->indexToCells(idx, idx_coord_x, idx_coord_y); 

    //Env: approx 8 x 7, costmap initialized at point (-2,-4), map size initialized to 64x64 due to gmapping auto map expansion issue.
    if(idx_coord_x > x_env_map_max_limit || idx_coord_y > y_env_map_max_limit || idx_coord_x < x_env_map_min_limit || idx_coord_y < y_env_map_min_limit) 
    {
      // std::cout << "**********************FS X coord: " << idx_coord_x << " size_x_ " << size_x_ << ", x_env_map_max_limit: " << x_env_map_max_limit << ", x_env_map_min_limit: " << x_env_map_min_limit << std::endl;
      // std::cout << "**********************FS Y coord: " << idx_coord_y << " size_y_ " << size_y_ << ", y_env_map_max_limit: " << y_env_map_max_limit << ", y_env_map_min_limit: " << y_env_map_min_limit << std::endl;
      continue; 
    }

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) 
    {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) 
      {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } 
      else if (isNewFrontierCell(nbr, frontier_flag)) 
      {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);

        if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) 
        {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }
  
  ROS_INFO("Total Frontiers before splitting = %d", int(frontier_list.size()));
  std::queue<Frontier> fq;
  for (int i=0; i<frontier_list.size(); i++)
  {
    fq.push(frontier_list[i]);
  }
  frontier_list.clear();

  while(fq.size() > 0)
  {
    Frontier frontier = fq.front();
    fq.pop();
    if (frontier.size * costmap_->getResolution() > max_frontier_size_*1.25)
    {
      //Split into two
      std::vector<Frontier> temp;
      temp = splitFrontierPCA(frontier, pos);
      fq.push(temp[0]);
      fq.push(temp[1]);
    }
    else
    {
      frontier_list.push_back(frontier);
    }
        
  }
  ROS_INFO("Total Frontiers after splitting = %d", int(frontier_list.size()));
  return frontier_list;
}


/**
 * Return the frontier with the maximum utility.
*/
std::vector<Frontier> FrontierSearch::getMaxUtilityFrontiers(std::vector<Frontier>& frontier_list,
                                                             quadmap::QuadMap& base_quadmap,
                                                             int& robot_id, bool use_relative_positions,
                                                             bool __FLAG_can_stop_now__,
                                                             std::vector<geometry_msgs::Point>& latest_relative_positions,
                                                             unsigned int x_env_map_max_limit, 
                                                             unsigned int y_env_map_max_limit,
                                                             unsigned int x_env_map_min_limit,
                                                             unsigned int y_env_map_min_limit)
{
  std::vector<Frontier> final_frontier_list;
  float f_cost_min = 100000, f_cost_max = 0;
  float info_gain_uexp_cell_count = 0;
  unsigned fmx, fmy;
  float info_used_at_frontier_percent = 0;;
  
  for(int i=0; i<frontier_list.size(); i++)
  {
    float info_gain_uexp_cell_count_max = -1;
    float info_used_at_frontier_percent_max = 0;
    Frontier frontier = frontier_list[i];
    info_gain_uexp_cell_count = 0;
    std::vector<geometry_msgs::Point> start_vec{frontier.centroid};
    std::vector<quadmap::Node> neighboring_robots_positions;
    bool Flag_use_view_points = true;
    int max_neighbor_count = 0;
    double beta_parameter = 1;

    

    if(Flag_use_view_points)
    {
      start_vec.push_back(frontier.initial);
      start_vec.push_back(frontier.end);
    }

    int choice = 0;
    int itr = 0;
    for(auto const start : start_vec)
    {
      itr+=1;

     
      // Leverage the latest relative position estimates and prefer forntiers that are further away form robots latest positions
      //Only use the distance to the closest robot to the frontier
      double dist_sum = 10000, dist_sum_avg=0;
      for(auto rel_val:latest_relative_positions)
      {
        dist_sum = std::min(dist_sum, (pow((rel_val.x-start.x),2) + pow((rel_val.y-start.y),2)));
      }
      
      // dist_sum_avg  /= int(latest_relative_positions.size());
      beta_parameter = log10(dist_sum);
      ROS_INFO("*************BetaParam : %f", beta_parameter);
      
      costmap_->worldToMap(start.x, start.y, fmx, fmy);
      unsigned int clear, frontier_pos  = costmap_->getIndex(fmx,fmy);
      
      ROS_INFO("***************************************************");
      // ROS_INFO("Frontier centroid World pos: %f, %f", frontier.centroid.x, frontier.centroid.y);
      ROS_INFO("Frontier Map pos: %d, %d", fmx, fmy);
      // ROS_INFO("Frontier centroid index= %d", frontier_pos);
      // ROS_INFO("Size of frontier: %f meters ", frontier.size * costmap_->getResolution());
      
      // bool val = nearestCellsWithinRange(uexp_cell_count, frontier_pos, NO_INFORMATION, FREE_SPACE,
      //                                    LETHAL_OBSTACLE, *costmap_, sensor_range_);
      
      //Get relative positions around a frontier centroid by searching the quadmap and update the vector neighboring_robots_positions
      
      // if(fmx > 40 || fmy > 36  || fmx < 10 || fmy < 4)
      if(fmx > x_env_map_max_limit || fmy > y_env_map_max_limit  || fmx < x_env_map_min_limit || fmy < y_env_map_min_limit)
      {
        info_gain_uexp_cell_count = 0;
      }
      else
      {      
        int ts=0;
        quadmap::Node center(fmx, fmy, robot_id, robot_id,ts); //Value of the 3rd parameter is meaningless here for the query
        neighboring_robots_positions.clear();
        if(use_relative_positions) //FLAG_WSR = true
        {
          base_quadmap.query_radius(center,2*sensor_range_,neighboring_robots_positions);
          // base_quadmap.query_radius(center,sensor_range_,neighboring_robots_positions);
        }
        info_used_at_frontier_percent = 0;
        ROS_INFO("Neighboring robot positions around the frontier = %ld", neighboring_robots_positions.size());
        bool val = InfoNearestCellsWithinRange(info_gain_uexp_cell_count, frontier_pos, NO_INFORMATION, *costmap_, sensor_range_,
                                              neighboring_robots_positions, 
                                              info_used_at_frontier_percent,
                                              x_env_map_max_limit, 
                                              y_env_map_max_limit,
                                              x_env_map_min_limit,
                                              y_env_map_min_limit);
      
        ROS_INFO("Info gain = %f", info_gain_uexp_cell_count);
        ROS_INFO("Info loss at the frontier (percent) = %f", info_used_at_frontier_percent);
      }

      if(beta_parameter*info_gain_uexp_cell_count > info_gain_uexp_cell_count_max)
      {
        info_gain_uexp_cell_count_max = beta_parameter*info_gain_uexp_cell_count;
        info_used_at_frontier_percent_max = info_used_at_frontier_percent;
        frontier.pos_id = frontier_pos;
        ROS_INFO("Info gain max updated = %f", info_gain_uexp_cell_count_max);
        max_neighbor_count = neighboring_robots_positions.size();
        choice=itr;
        frontier.view_point_to_navigate_to = start;
      }

    }

    
    ROS_INFO("****** Max Info gain around frontier: %f ***********", info_gain_uexp_cell_count_max);
    if(choice == 1) 
    {
      ROS_INFO("Navigating to centroid of frontier");
    }
    else if(choice == 2) 
    {
      ROS_INFO("Navigating to left extreme of the frontier");
    }
    else if(choice == 3) 
    {
      ROS_INFO("Navigating to right extreme of the frontier");
    }

    frontier.cost = frontierUtility(frontier, info_gain_uexp_cell_count_max); //New cost function
    if(frontier.cost > 0)
    {
      frontier.neighbors_count = max_neighbor_count;
      frontier.information_gain = info_gain_uexp_cell_count_max;
      frontier.info_used_percent = info_used_at_frontier_percent_max;

      // if(frontier.cost > 1)
      // {
      //   final_frontier_list.push_back(frontier);
      // }
      // if(__FLAG_can_stop_now__ && frontier.information_gain < 10)
      
      if(__FLAG_can_stop_now__ && frontier.info_used_percent >= 90.0)
      // if(__FLAG_can_stop_now__ && frontier.info_used_percent >= 50.0) //For hardware experiments in the flight lab
      // if(__FLAG_can_stop_now__ && frontier.cost < 5)
      {
        ROS_INFO("Discarding frontier");
      }
      else
      {
        final_frontier_list.push_back(frontier);
      }
    }
  }


  // For frontier Utility, the frontier with highest utility should be the first
  std::sort(
    final_frontier_list.begin(), final_frontier_list.end(),
    [](const Frontier& f1, const Frontier& f2) { return f1.cost > f2.cost; });

  return final_frontier_list;
}

/**
 * @brief Build a new frontier
 * */
Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);// This stores all the positions of the points/cells within a frontier.

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  
  //This distance is already in world coordinates.
  output.centroid_distance = sqrt(pow((double(reference_x) - double(output.centroid.x)), 2.0) +
                                  pow((double(reference_y) - double(output.centroid.y)), 2.0));
  
  
  std::vector<geometry_msgs::Point> extreme_vals = getFrontierSegmentExtremes(output.points, output.centroid);
  
  output.initial = extreme_vals[0];
  output.end = extreme_vals[1];
  
  return output;
}


double FrontierSearch::calculateYawAngle(double x1, double y1, double x2, double y2) 
{
    // Calculate the difference in coordinates
    double deltaX = x2 - x1;
    double deltaY = y2 - y1;

    // Calculate the angle in radians
    double angleRadians = atan2(deltaY, deltaX);

    // Convert the angle to degrees
    double angleDegrees = angleRadians * (180.0 / M_PI);

    return angleDegrees;
}


/*
* Get the exteme points of the frontier.
*/
std::vector<geometry_msgs::Point> FrontierSearch::getFrontierSegmentExtremes
                                                    (std::vector<geometry_msgs::Point>Points, 
                                                    geometry_msgs::Point centroid) 
{   
    double maxYawAngle = -180;
    double minYawAngle = 180;
    geometry_msgs::Point left_extreme;
    geometry_msgs::Point right_extreme;

    for (const auto& point : Points) 
    {
      double yawAngle = calculateYawAngle(centroid.x, centroid.y, point.x, point.y);
      // ROS_INFO("Yaw angele %f deg", yawAngle);  

      if(yawAngle > maxYawAngle)
      { 
        maxYawAngle = yawAngle;
        left_extreme = point;
      }
      
      if(yawAngle <= minYawAngle)
      { 
        minYawAngle = yawAngle;
        right_extreme = point;
      }
    }

    // ROS_INFO("Yaw angele max - left: %f deg, min - right: %f deg", maxYawAngle, minYawAngle);
    std::vector<geometry_msgs::Point> output{left_extreme, right_extreme};
    return output;
}


/**
 * @brief Check if a cell is a new frontier cell
 * */
bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}


/**
 * @brief Original code for frontier costcomputation
 * */
double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance * costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}


/**
 * Greedy method, Single robot
 * */
double FrontierSearch::frontierUtility(const Frontier& frontier,
                                      float& information_gain)
{
  double res;

  //Note: the centroid distance does not account for the map resoulution, but since its just a scaler multiplier. 
  // res = (alpha_parameter_*frontier.size *information_gain)/(beta_parameter_*frontier.centroid_distance);
  if(frontier.centroid_distance == 0) 
    res = information_gain;
  else
    res = (information_gain)/(frontier.centroid_distance);

  ROS_INFO("Information gain based on unexplored cell count and beta parameter = %f", information_gain);
  ROS_INFO("frontier centroid distance = %f", frontier.centroid_distance);

  

  ROS_INFO("frontier Utility = %f", res);
  ROS_INFO("***************************************************");  
  return res;
}


/**
 * Greedy method multi robot
 * Deprecated
 * */
// double FrontierSearch::frontierUtility(const Frontier& frontier,
//                                       float& information_gain,
//                                       std::vector<geometry_msgs::Point> rel_positions, 
//                                       float& effort)
// {
//   ROS_INFO("Total Information gain (accounting for sensor overlap) = %f", information_gain);

//   float min_dist_j_all = 100000, temp=0;
//   for(int j=0; j<rel_positions.size(); j++)
//   {
    
//     //Change temp to the closest distance to the frontier
//     float min_dist_j = 100000;
//     for(int tau=0; tau<frontier.points.size(); tau++)
//     {
//       temp = sqrt(pow((rel_positions[j].x - frontier.points[tau].x), 2.0) +
//                   pow((rel_positions[j].y - frontier.points[tau].y), 2.0));

//       min_dist_j = min_dist_j < temp ? min_dist_j: temp;
//     }
    
//     min_dist_j_all = min_dist_j_all < min_dist_j  ? min_dist_j_all : min_dist_j ;
//   }

//   int gamma_1 = 1;
//   float gamma_2 =  min_dist_j_all <= frontier.centroid_distance ? (1/(1+min_dist_j_all)) : 0;
//   effort = gamma_1 * frontier.centroid_distance * (1 + gamma_2);
//   auto val = information_gain / effort;
//   // effort = frontier.centroid_distance; //Added only for testing;
//   ROS_INFO("frontier effort of robot i = %f", frontier.centroid_distance);
//   ROS_INFO("frontier Utility for robot i = %f", val);

//   return val;
// }

/**
 * @brief Compute distance to neighboring robot. Might be removed in future
 * */
float FrontierSearch::neighborhoodDistance(unsigned int start,
                                          std::vector<geometry_msgs::Point> rel_positions)
{
  unsigned int sx, sy;
  double wx, wy;
  float total_dist_j = 0;

  //This is redundant calculation and done in multiple functions. Optimize later
  costmap_->indexToCells(start, sx, sy);
  costmap_->mapToWorld(sx, sy, wx, wy); //World coordinates of the frontier centroid
  for(int j=0; j<rel_positions.size(); j++)
  {
    total_dist_j += sqrt(pow((rel_positions[j].x-wx),2) + pow((rel_positions[j].y-wy),2));
  }

  return total_dist_j;
}

/**
 * @DEPRECATED
 * Splits a large frontier into two smaller frontiers
 * */
std::vector<Frontier> FrontierSearch::splitFrontier(const Frontier& frontier,
                                                    unsigned int reference)
{
  int start = 0;
  int middle = int(frontier.size/2);
  std::vector<Frontier> temp;
  unsigned fmx, fmy;
  
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  unsigned int size_x_ = costmap_->getSizeInCellsX(),
               size_y_ = costmap_->getSizeInCellsY(),
               map_size = size_x_ * size_y_;

  for(int i=0; i<2; i++)
  {
    Frontier current;
    current.centroid.x = 0;
    current.centroid.y = 0;
    double wx, wy;
    current.size = 1;
    current.min_distance = std::numeric_limits<double>::infinity();
    // ROS_INFO("start = %d, end = %d", start, middle);
    for(int j=start; j<middle; j++)
    {
      current.points.push_back(frontier.points[j]);// This stores all the positions of the points/cells within a frontier.
      wx = frontier.points[j].x;
      wy = frontier.points[j].y;

      // update frontier size
      current.size++;

      // update centroid of frontier
      costmap_->worldToMap(wx, wy, fmx, fmy);
      unsigned int clear, f_point_idx  = costmap_->getIndex(fmx,fmy); 
      if(f_point_idx < map_size-1 ) //Check added to handle incorrect coordinate bug
      {
	      current.centroid.x += wx;
        current.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                                pow((double(reference_y) - double(wy)), 2.0));
        if (distance < current.min_distance) {
          current.min_distance = distance;
          current.middle.x = wx;
          current.middle.y = wy;
        }
      }
      else{
	        ROS_INFO("Out of bounds detected and thrown away");	
      };

    }

    // average out frontier centroid
    current.centroid.x /= current.size;
    current.centroid.y /= current.size;


    //centroid == middle point of the frontier
    // current.centroid.x = current.points[int(current.size/2)].x;
    // current.centroid.y = current.points[int(current.size/2)].y;

    //This distance is already in world coordinates.
    current.centroid_distance = sqrt(pow((double(reference_x) - double(current.centroid.x)), 2.0) +
                                    pow((double(reference_y) - double(current.centroid.y)), 2.0));

    temp.push_back(current);
    start = middle+1;
    middle = int(frontier.size);
  }

  return temp;
}


/**
 * Reference: https://github.com/HKUST-Aerial-Robotics/FUEL
 * Splits a large frontier into two smaller frontiers using PCA
 * */
std::vector<Frontier> FrontierSearch::splitFrontierPCA(const Frontier& frontier,
                                                      unsigned int reference)
{
  Eigen::Matrix2d cov;
  cov.setZero();
  
  // Covariance matrix of cells
  for (auto cell : frontier.points) 
  {
    Eigen::Vector2d diff(0,0); 
    diff[0] = cell.x - frontier.centroid.x; 
    diff[1] = cell.y - frontier.centroid.y;
    cov += diff * diff.transpose(); ;
  }
  cov /= double(frontier.size);

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) 
  {
    if (values[i] > max_eigenvalue) 
    {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  std::cout << "max idx: " << max_idx << std::endl;
  std::cout << "first pc: " << first_pc.transpose() << std::endl;


  std::vector<Frontier> temp;
  Frontier split1, split2;

  for(auto cell : frontier.points)
  {
    Eigen::Vector2d diff(0,0); 
    diff[0] = cell.x - frontier.centroid.x; 
    diff[1] = cell.y - frontier.centroid.y;
    if(diff.dot(first_pc) >= 0)
    {
      split1.points.push_back(cell);
    }
    else
    {
      split2.points.push_back(cell);
    }
  }

  updateInfo(split1, reference);
  updateInfo(split2, reference);

  temp.push_back(split1);
  temp.push_back(split2);

  return temp;
}

/**
 * Update the information of the frontiers.
 * */
void FrontierSearch::updateInfo(Frontier& frontier,
                                unsigned int reference)
  {
    unsigned int rx, ry;
    double reference_x, reference_y;
    costmap_->indexToCells(reference, rx, ry);
    costmap_->mapToWorld(rx, ry, reference_x, reference_y);

    frontier.centroid.x = 0;
    frontier.centroid.y = 0;
    double wx, wy;
    frontier.size = frontier.points.size();
    frontier.min_distance = std::numeric_limits<double>::infinity();
    for (auto cell : frontier.points)
    {
      wx = cell.x;
      wy = cell.y;

      // update centroid of frontier
      frontier.centroid.x += wx;
      frontier.centroid.y += wy;

      // determine frontier's distance from robot, going by closest gridcell
      // to robot
      double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                            pow((double(reference_y) - double(wy)), 2.0));
      if (distance < frontier.min_distance) 
      {
        frontier.min_distance = distance;
        frontier.middle.x = wx;
        frontier.middle.y = wy;
      }
    }
    // average out frontier centroid
    frontier.centroid.x /= frontier.size;
    frontier.centroid.y /= frontier.size;

    frontier.centroid_distance = sqrt(pow((double(reference_x) - double(frontier.centroid.x)), 2.0) +
                                    pow((double(reference_y) - double(frontier.centroid.y)), 2.0));
  }
}
