#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#define _USE_MATH_DEFINES

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <cmath>
#include <explore/quadmap.h>

namespace frontier_exploration
{
  /**
   * @brief Determine 4-connected neighbourhood of an input cell, checking for map
   * edges
   * @param idx input cell index
   * @param costmap Reference to map data
   * @return neighbour cell indexes
   */
  std::vector<unsigned int> nhood4(unsigned int idx,
                                  const costmap_2d::Costmap2D& costmap)
  {
    // get 4-connected neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out;

    unsigned int size_x_ = costmap.getSizeInCellsX(),
                size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ - 1) {
      ROS_WARN("Evaluating nhood for offmap point");
      return out;
    }

    if (idx % size_x_ > 0) {
      out.push_back(idx - 1);
    }
    if (idx % size_x_ < size_x_ - 1) {
      out.push_back(idx + 1);
    }
    if (idx >= size_x_) {
      out.push_back(idx - size_x_);
    }
    if (idx < size_x_ * (size_y_ - 1)) {
      out.push_back(idx + size_x_);
    }
    return out;
  }

  /**
   * @brief Determine 8-connected neighbourhood of an input cell, checking for map
   * edges
   * @param idx input cell index
   * @param costmap Reference to map data
   * @return neighbour cell indexes
   */
  std::vector<unsigned int> nhood8(unsigned int idx,
                                  const costmap_2d::Costmap2D& costmap)
  {
    // get 8-connected neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out = nhood4(idx, costmap);

    unsigned int size_x_ = costmap.getSizeInCellsX(),
                size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ - 1) {
      return out;
    }

    if (idx % size_x_ > 0 && idx >= size_x_) {
      out.push_back(idx - 1 - size_x_);
    }
    if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
      out.push_back(idx - 1 + size_x_);
    }
    if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
      out.push_back(idx + 1 - size_x_);
    }
    if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
      out.push_back(idx + 1 + size_x_);
    }

    return out;
  }

  /**
   * @brief Find nearest cell of a specified value
   * @param result Index of located cell
   * @param start Index initial cell to search from
   * @param val Specified value to search for
   * @param costmap Reference to map data
   * @return True if a cell with the requested value was found
   */
  bool nearestCell(unsigned int& result, unsigned int start, unsigned char val,
                  const costmap_2d::Costmap2D& costmap)
  {
    const unsigned char* map = costmap.getCharMap();
    const unsigned int size_x = costmap.getSizeInCellsX(),
                      size_y = costmap.getSizeInCellsY();

    if (start >= size_x * size_y) {
      return false;
    }

    // initialize breadth first search
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(size_x * size_y, false);

    // push initial cell
    bfs.push(start);
    visited_flag[start] = true;

    // search for neighbouring cell matching value
    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();

      // return if cell of correct value is found
      if (map[idx] == val) {
        result = idx;
        return true;
      }

      // iterate over all adjacent unvisited cells
      for (unsigned nbr : nhood8(idx, costmap)) {
        if (!visited_flag[nbr]) {
          bfs.push(nbr);
          visited_flag[nbr] = true;
        }
      }
    }

    return false;
  }

  /**
   * @brief Find the information gain from all unknown cells (using sigmoid function) within a sensor range of 'start' cell
   * @param result Count of such cells
   * @param start Index initial cell to search from
   * @param val Specified value to search for
   * @param costmap Reference to map data
   * @param range Range to search within
   * @param neighboring_robot_status if the robot is esimated to be dead or alive
   * @param relative_position_est_trace_omega omega based on the trace of the covariance matrix relative position estimates
   * @param rel_pos_neighbors relative position estimates of the neighboring robots
   * @param info_used_at_frontier_percent Indicates amount of info left
   * @return True if a cell with the requested value was found
   */
  bool InfoNearestCellsWithinRange(float& result, unsigned int start, unsigned char cell_val,
                                   const costmap_2d::Costmap2D& costmap, double& sensor_range,
                                   std::vector<quadmap::Node>& neighboring_robots_quadmap_positions,
                                   float& info_used_at_frontier_percent,
                                   unsigned int x_env_map_max_limit, 
                                   unsigned int y_env_map_max_limit,
                                   unsigned int x_env_map_min_limit,
                                   unsigned int y_env_map_min_limit)
  {
    const unsigned char* map = costmap.getCharMap();
    const unsigned int size_x = costmap.getSizeInCellsX(),
                      size_y = costmap.getSizeInCellsY();

    float dist_i = 0;
    float dist_j = 0;
    float sigmoid_cost_midpoint_ = sensor_range*0.5; //Note that since the sigmoid is still based on the sensor range only
    float sigmoid_cost_steepness_ = 0.1; 
    float sigmoid_cost_amplitude_ = 1.0;
    float E_hat_c = 0;
    float S_c = 0;
    float info_loss_with_distance = 0, info_loss_with_time=0;
    unsigned int sx, sy, nx, ny;
    double swx, swy, wx, wy, rwx, rwy;
    float L = sensor_range;
    int rel_positions_in_known_region=0;
    float total_info_from_a_frontier = 0.01; //to avoid division by zero
    float info_loss_at_a_frontier = 0;

    if (start >= size_x * size_y) 
    {
      return false;
    }

    /* Just for logging
    //Sort the relative position nodes based on the omega
    std::cout << "======== Before sorting ============" << std::endl;
    for(int jjj = 0; jjj < neighboring_robots_quadmap_positions.size(); jjj++)
    {
      std::cout << neighboring_robots_quadmap_positions[jjj].omega << std::endl;
    }
    
    std::sort(
    neighboring_robots_quadmap_positions.begin(), neighboring_robots_quadmap_positions.end(),
    [](const quadmap::Node& n1, const quadmap::Node& n2) { return n1.omega > n2.omega;});


    std::cout << "======== After sorting ============" << std::endl;
    //Sort the relative position nodes based on the omega
    for(int jjj = 0; jjj < neighboring_robots_quadmap_positions.size(); jjj++)
    {
      std::cout << neighboring_robots_quadmap_positions[jjj].omega << std::endl;
    }
    */
    
    // initialize breadth first search
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(size_x * size_y, false);

    // push initial cell
    bfs.push(start);
    visited_flag[start] = true;
    costmap.indexToCells(start, sx, sy);
    costmap.mapToWorld(sx, sy, swx, swy);

    // search for neighbouring cell matching value and have not been already marked as visited by other robots.
    while (!bfs.empty()) 
    {
      unsigned int idx = bfs.front();
      bfs.pop();
      E_hat_c = 0;
      S_c = 0;

      // return if cell of correct value is found
      if (map[idx] == cell_val) 
      {
        costmap.indexToCells(idx, nx, ny);
        costmap.mapToWorld(nx, ny, wx, wy); //Need everything in world coordinates

        dist_i = sqrt(pow((swx-wx),2) + pow((swy-wy),2));
        S_c = sigmoid_cost_amplitude_ * 1/(1+exp((dist_i-sigmoid_cost_midpoint_)/sigmoid_cost_steepness_));
        
        //Info available from a cell decays with distance. 
        //Using sigmoid based on DARPA IROS 2022 papers. 
        //Basically we want to capture the uncertainty in information gain as the cell distance increases from a frontier
        //Subtract the information loss due to other robots positions 
        
        // int iterator = 0;
        
        for (auto neighboring_robot_val : neighboring_robots_quadmap_positions) 
        {
            costmap.mapToWorld(neighboring_robot_val.est_mx, neighboring_robot_val.est_my, rwx, rwy);
            // costmap.mapToWorld(neighboring_robot_val.true_mx, neighboring_robot_val.true_my, rwx, rwy);
            dist_j = sqrt(pow((rwx-wx),2) + pow((rwy-wy),2));
            
            /*Omega incorporates the uncertainty in the robot j position estimate computation using the trace of the covariance matrix.*/
            info_loss_with_distance =  neighboring_robot_val.omega * sigmoid_cost_amplitude_ * 1/(1+exp((dist_j-sigmoid_cost_midpoint_)/sigmoid_cost_steepness_));
            
            /*Tau = 1 indicates that the robot j is still operational*/
            E_hat_c += neighboring_robot_val.getTau() * info_loss_with_distance; //Check whether to include the loss due to a robot (e.g. only when its functional)
        }

        // ROS_INFO("--------------------------------------------------");
        // if(neighboring_robots_quadmap_positions.size()>0) 
        // {
        //   E_hat_c /= int(neighboring_robots_quadmap_positions.size()); //Average out the loss
        // }
        
        /*Info cannot be negative. Hence do not consider negative values. 
          The lowest info from a frontier should be 0 and not negative.
          The RHS or line 261 is thus interpreted as net info of a cell
        */
        result += std::max(0.0, double(S_c - E_hat_c));
        total_info_from_a_frontier+= S_c; // Total info of a cell
        info_loss_at_a_frontier+= S_c - std::max(0.0, double(S_c - E_hat_c)); // Total - net 
      }

      // iterate over all adjacent unvisited cells which are withing range from start cell (sx, sy)
      for (unsigned nbr : nhood8(idx, costmap)) 
      {
        if (!visited_flag[nbr]) 
        {
          costmap.indexToCells(nbr, nx, ny);

          //CHECKS IF CELL IS WITHIN BOUNDS For the explorer tb3 robots gmapping config.
          if(nx > x_env_map_max_limit || ny > y_env_map_max_limit  || nx < x_env_map_min_limit || ny < y_env_map_min_limit) {
            // std::cout << "********************** X coord: " << nx << " size_x " << size_x << std::endl;
            // std::cout << "********************** Y coord: " << ny << " size_y " << size_y << std::endl;
            continue; 
          }
          costmap.mapToWorld(nx, ny, wx, wy);
          dist_i = sqrt(pow((swx-wx),2) + pow((swy-wy),2)); 
          if(dist_i <= sensor_range) 
          {
              bfs.push(nbr);
          }
          visited_flag[nbr] = true;
        }
      }
    }

    ROS_INFO("info_loss_at_a_frontier E_hat_c: %f", info_loss_at_a_frontier);
    ROS_INFO("total_info_from_a_frontier S_c: %f", total_info_from_a_frontier);
    info_used_at_frontier_percent = std::min(float(100.0), float(100*info_loss_at_a_frontier/total_info_from_a_frontier)); //Just for logging
    
    return true;
  }


}

#endif
