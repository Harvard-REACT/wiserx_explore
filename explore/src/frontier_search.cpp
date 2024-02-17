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
                               double min_frontier_size, double range, float decay_rate,
                               float utility_alpha_parameter, float utility_beta_parameter)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , sensor_range_(range)
  , decay_rate_(decay_rate)
  , alpha_parameter_(utility_alpha_parameter)
  , beta_parameter_(utility_beta_parameter)
{
  //Calculate max size permissible for any frontier. Frontiers larger than this will be broken down
  max_frontier_size_ = (2*M_PI*sensor_range_) / (costmap_->getResolution() * 3);
}

/**
 * @brief Original frontier search function
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
  } else {
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
 * @brief Information gain computation without using relative position of neighbors for utility computation
 * */
std::vector<Frontier> FrontierSearch::searchFromNew(geometry_msgs::Point position,
                                                    std::vector<geometry_msgs::Point> relative_position_of_neighboring_robots)
{
  std::vector<Frontier> frontier_list, final_frontier_list;
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
  

  /* Break large frontiers into smaller frontiers */
  ROS_INFO("Total Frontiers = %d", frontier_list.size());
  // ROS_INFO("Max Frontier size = %d", max_frontier_size_);

  std::queue<Frontier> fq, new_fq;

  for (int i=0; i<frontier_list.size(); i++)
  {
    // fq.push(frontier_list[i]);
    new_fq.push(frontier_list[i]);
  }
  frontier_list.clear();


  // while(fq.size() > 0)
  // {
  //   Frontier frontier = fq.front();
  //   //ROS_INFO("Frontier list size = %d", fq.size()); 
  //   //ROS_INFO("Current Frontier size = %d", frontier.size);
  //   if (frontier.size > max_frontier_size_)
  //   {
  //     //Split into two
  //     std::vector<Frontier> temp;
  //     //temp = splitFrontier(frontier, pos);
  //     temp = splitFrontierPCA(frontier, pos);
  //     // ROS_INFO("temp[0] = %d, temp[1] = %d", temp[0].points.size(), temp[1].points.size());
  //     fq.push(temp[0]);
  //     fq.push(temp[1]);
  //   }
  //   else
  //   {
  //     new_fq.push(frontier);
  //   }
  //   // new_fq.push(frontier);
  //   // ROS_INFO("Frontier list size after splitting = %d", fq.size());
  //   fq.pop();
  //   // ROS_INFO("Frontier list size after removing the first = %d", fq.size());
    
  // }
  // ROS_INFO("Total Frontiers after splitting = %d", new_fq.size());


  /* Compute Utility */
  float info_gain_uexp_cell_count = 0;
  unsigned fmx, fmy;
  while(!new_fq.empty())
  {
    Frontier frontier = new_fq.front();
    new_fq.pop();
    info_gain_uexp_cell_count = 0;
    costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, fmx, fmy);
    unsigned int clear, frontier_pos  = costmap_->getIndex(fmx,fmy);
    
    ROS_INFO("***************************************************");
    ROS_INFO("Frontier centroid World pos: %f, %f", frontier.centroid.x, frontier.centroid.y);
    ROS_INFO("Frontier centroid Map pos: %d, %d", fmx, fmy);
    ROS_INFO("Frontier centroid index= %d", frontier_pos);
    
    // bool val = nearestCellsWithinRange(uexp_cell_count, frontier_pos, NO_INFORMATION, FREE_SPACE,
    //                                    LETHAL_OBSTACLE, *costmap_, sensor_range_);
    
    bool val = InfoNearestCellsWithinRange(info_gain_uexp_cell_count, frontier_pos, NO_INFORMATION, *costmap_, sensor_range_);
  
    // ROS_INFO("****** Unexplored cells around frontier: %d ***********", uexp_cell_count);
    ROS_INFO("Size of frontier: %f meters ", frontier.size * costmap_->getResolution());

    // frontier.cost = frontierCost(frontier); //Default cost function
    frontier.cost = frontierUtility(frontier, info_gain_uexp_cell_count); //New cost function, single robot
    
    if(info_gain_uexp_cell_count == 0) continue;

    frontier.information_gain = info_gain_uexp_cell_count;
    frontier.effort = frontier.centroid_distance;
    frontier.pos_id = frontier_pos;
    frontier.neighbor_distance = neighborhoodDistance(frontier_pos, relative_position_of_neighboring_robots);
    frontier.neighbors = relative_position_of_neighboring_robots.size();
    final_frontier_list.push_back(frontier);
  }

  // For frontier Utility, the frontier with highest utility should be the first
  std::sort(
    frontier_list.begin(), frontier_list.end(),
    [](const Frontier& f1, const Frontier& f2) { return f1.cost > f2.cost; });

  return final_frontier_list;
}



/**
 * New formulation 4
*/
std::vector<Frontier> FrontierSearch::searchFromWithNeighorInfo(geometry_msgs::Point position,
                                                                std::vector<geometry_msgs::Point> relative_position_of_neighboring_robots)
{
  std::vector<Frontier> frontier_list, final_frontier_list;
  float f_cost_min = 100000, f_cost_max = 0;
  float information_gain=0, effort = 0;
  unsigned fmx, fmy;
  int uexp_cell_count = 0;

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
  } else {
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


  //Break large frontiers into smaller frontiers
  ROS_INFO("Total Frontiers = %d", frontier_list.size());
  std::queue<Frontier> fq, new_fq;

  for (int i=0; i<frontier_list.size(); i++)
  {
    fq.push(frontier_list[i]);
  }
  frontier_list.clear();


  while(fq.size() > 0)
  {
    Frontier frontier = fq.front();
    // ROS_INFO("Frontier list size = %d", fq.size()); 
    ROS_INFO("Current Frontier size = %d", frontier.size);
    ROS_INFO("Max Frontier size = %d", max_frontier_size_);
    if (frontier.size > max_frontier_size_)
    {
      //Split into two
      std::vector<Frontier> temp;
      // temp = splitFrontier(frontier, pos);
      temp = splitFrontierPCA(frontier, pos);
      ROS_INFO("temp[0] = %d, temp[1] = %d", temp[0].points.size(), temp[1].points.size());
      fq.push(temp[0]);
      fq.push(temp[1]);
    }
    else
    {
      new_fq.push(frontier);
    }
    
    // ROS_INFO("Frontier list size after splitting = %d", fq.size());
    fq.pop();
    // ROS_INFO("Frontier list size after removing the first = %d", fq.size());
    
  }
  ROS_INFO("Total Frontiers after splitting = %d", new_fq.size());
  



  // set travel and information gain costs of frontiers
  while(!new_fq.empty())
  // for (auto& frontier : final_frontier_list) 
  {
    Frontier frontier = new_fq.front();
    new_fq.pop();
    information_gain = 0;
    effort = 0;
    uexp_cell_count = 0;
    
    
    costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, fmx, fmy);
    unsigned int clear, frontier_pos  = costmap_->getIndex(fmx,fmy);
    
    ROS_INFO("***************************************************");
    ROS_INFO("Frontier centroid World pos: %f, %f", frontier.centroid.x, frontier.centroid.y);
    ROS_INFO("Frontier centroid Map pos: %d, %d", fmx, fmy);
    ROS_INFO("Frontier centroid index= %d", frontier_pos);

    /*Added here only for testing*/    
    // bool val = nearestCellsWithinRange(uexp_cell_count, frontier_pos, NO_INFORMATION, FREE_SPACE,
    //                                    LETHAL_OBSTACLE, *costmap_, sensor_range_);
    // information_gain = uexp_cell_count;
    /*******/

    informationGain(information_gain, frontier_pos, NO_INFORMATION,
                    *costmap_, sensor_range_, total_overlap_g_c_, decay_rate_,
                    relative_position_of_neighboring_robots, eta_);

    // ROS_INFO("****** Unexplored cells around frontier: %d ***********", uexp_cell_count);
    ROS_INFO("Size of frontier: %d ", frontier.size);

    frontier.cost = frontierUtility(frontier, information_gain, relative_position_of_neighboring_robots, effort);
    
    if(information_gain == 0) continue;
    
    frontier.information_gain = information_gain;
    frontier.effort = effort;
    frontier.pos_id = frontier_pos;
    frontier.neighbor_distance = neighborhoodDistance(frontier_pos, relative_position_of_neighboring_robots);
    frontier.neighbors = relative_position_of_neighboring_robots.size();
    final_frontier_list.push_back(frontier);
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
  res = (alpha_parameter_*frontier.size *information_gain)/(beta_parameter_*frontier.centroid_distance);
  ROS_INFO("Information gain based on unexplored cell count = %f", information_gain);
  ROS_INFO("frontier centroid distance = %f", frontier.centroid_distance);
  ROS_INFO("frontier Utility = %f", res);
  
  return res;
}

/**
 * Greedy method multi robot
 * */
double FrontierSearch::frontierUtility(const Frontier& frontier,
                                      float& information_gain,
                                      std::vector<geometry_msgs::Point> rel_positions, 
                                      float& effort)
{
  ROS_INFO("Total Information gain (accounting for sensor overlap) = %f", information_gain);

  float min_dist_j_all = 100000, temp=0;
  for(int j=0; j<rel_positions.size(); j++)
  {
    
    //Change temp to the closest distance to the frontier
    float min_dist_j = 100000;
    for(int tau=0; tau<frontier.points.size(); tau++)
    {
      temp = sqrt(pow((rel_positions[j].x - frontier.points[tau].x), 2.0) +
                  pow((rel_positions[j].y - frontier.points[tau].y), 2.0));

      min_dist_j = min_dist_j < temp ? min_dist_j: temp;
    }
    
    min_dist_j_all = min_dist_j_all < min_dist_j  ? min_dist_j_all : min_dist_j ;
  }

  int gamma_1 = 1;
  float gamma_2 =  min_dist_j_all <= frontier.centroid_distance ? (1/(1+min_dist_j_all)) : 0;
  effort = gamma_1 * frontier.centroid_distance * (1 + gamma_2);
  auto val = information_gain / effort;
  // effort = frontier.centroid_distance; //Added only for testing;
  ROS_INFO("frontier effort of robot i = %f", frontier.centroid_distance);
  ROS_INFO("frontier Utility for robot i = %f", val);

  return val;
}

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
