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
   * @brief NJ addition - Find the information gain from all unknown cells (using sigmoid function) within a sensor range of 'start' cell
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
                                   std::vector<quadmap::Node>& neighboring_robots_positions,
                                   float& info_used_at_frontier_percent)
  {
    const unsigned char* map = costmap.getCharMap();
    const unsigned int size_x = costmap.getSizeInCellsX(),
                      size_y = costmap.getSizeInCellsY();

    float dist_i = 0;
    float dist_j = 0;
    float sigmoid_cost_midpoint_ = sensor_range*0.5; //Note that since the sigmoid is still based on the sensor range only
    // float sigmoid_cost_steepness_ = 0.1; //0.75 //0.1 ==> almost uniform (for older sigmoid formualation)
    float sigmoid_cost_steepness_ = 0.1; 
    float sigmoid_cost_amplitude_ = 1.0;
    float E_hat_c = 0;
    float info_loss_with_distance = 0, info_loss_with_time=0;
    unsigned int sx, sy, nx, ny;
    double swx, swy, wx, wy, rwx, rwy;
    float L = sensor_range;
    int rel_positions_in_known_region=0;
    float total_info_from_a_frontier = 0;
    float info_loss_at_a_frontier = 0;

    if (start >= size_x * size_y) 
    {
      return false;
    }

    // //Sort the relative position nodes based on the omega
    // std::cout << "======== Before sorting ============" << std::endl;
    // for(int jjj = 0; jjj < neighboring_robots_positions.size(); jjj++)
    // {
    //   std::cout << neighboring_robots_positions[jjj].omega << std::endl;
    // }
    
    // std::sort(
    // neighboring_robots_positions.begin(), neighboring_robots_positions.end(),
    // [](const quadmap::Node& n1, const quadmap::Node& n2) { return n1.omega > n2.omega;});


    // std::cout << "======== After sorting ============" << std::endl;
    // //Sort the relative position nodes based on the omega
    // for(int jjj = 0; jjj < neighboring_robots_positions.size(); jjj++)
    // {
    //   std::cout << neighboring_robots_positions[jjj].omega << std::endl;
    // }
    
    // initialize breadth first search
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(size_x * size_y, false);

    // push initial cell
    bfs.push(start);
    visited_flag[start] = true;
    costmap.indexToCells(start, sx, sy);
    costmap.mapToWorld(sx, sy, swx, swy);

    //NJ - This doesn't work.
    // if(neighboring_robots_positions.size() > 0)
    // {
    //   unsigned int idx_rel_position;
    //   for (auto neighboring_robot_val : neighboring_robots_positions) 
    //   {
    //     costmap.mapToWorld(neighboring_robot_val.est_mx, neighboring_robot_val.est_my, rwx, rwy);
    //     dist_j = sqrt(pow((rwx-swx),2) + pow((rwy-swy),2));
        
    //     if(dist_j <= sensor_range) //Checking closer proximity than when computing overlap
    //     {
    //       ROS_INFO("[Frontier Info gain] Found a position in known region at distance %f", dist_j);
    //       idx_rel_position = costmap.getIndex(neighboring_robot_val.est_mx, neighboring_robot_val.est_my);
    //       if(map[idx_rel_position] != cell_val) rel_positions_in_known_region+=1;
    //     }
    //   }
    
    //   //If 90% of positions around a frontier are in known space, it likely indicates that the region has been explored throughly by other robots
    //   //and won't have high information due to proximity of a region boundary. Hence assign 0 information gain to it.
    //   if(rel_positions_in_known_region/int(neighboring_robots_positions.size()) >= 0.5) 
    //   {
    //     ROS_INFO("[Frontier Info gain] Highly explored region (rel positions in known : %f %), hence assigning 0 info gain", (rel_positions_in_known_region/neighboring_robots_positions.size())*100);
    //     result = 0; 
    //     return true;
    //   }
    // }

    // std::sort(
    //   neighboring_robots_positions.begin(), neighboring_robots_positions.end(),
    //   [](const quadmap::Node& n1, const quadmap::Node& n2) { return n1.timestep < n1.timestep; });


    // ROS_INFO("--------------------------------------------------");
    // for (auto neighboring_robot_val : neighboring_robots_positions) 
    // {
    //   ROS_INFO("Timestep : %d ", neighboring_robot_val.timestep);
    // }

    //Mark all cells withing sensing range of the relative position of other robots as visited.
    // if(neighboring_robots_positions.size() > 0)
    // {
    //   unsigned int idx_rel_position;
    //   for (auto neighboring_robot_val : neighboring_robots_positions) 
    //     {
    //       idx_rel_position = costmap.getIndex(neighboring_robot_val.true_mx, neighboring_robot_val.true_my);
    //       costmap.mapToWorld(neighboring_robot_val.true_mx, neighboring_robot_val.true_my, rwx, rwy);
    //       std::queue<unsigned int> bfs_rel;
    //       bfs_rel.push(idx_rel_position);

    //       while(!bfs_rel.empty())
    //       {
    //         unsigned int idx_near_rel = bfs.front();
    //         bfs_rel.pop();
    //         for (unsigned nbr : nhood8(idx_near_rel, costmap)) 
    //         {
    //           if (!visited_flag[nbr]) 
    //           {
    //             costmap.indexToCells(nbr, nx, ny);
    //             costmap.mapToWorld(nx, ny, wx, wy);
    //             dist_i = sqrt(pow((rwx-rwy),2) + pow((swy-wy),2)); 
    //             if(dist_i <= sensor_range) 
    //             {
    //                 bfs_rel.push(nbr);
    //             }
    //             visited_flag[nbr] = true;     //Ideally we mark them true if the information loss is beyond a threshold.
    //           }
    //         }
    //       }
    //   }
    // }


    // search for neighbouring cell matching value and have not been already marked as visited by other robots.
    while (!bfs.empty()) 
    {
      unsigned int idx = bfs.front();
      bfs.pop();

      // return if cell of correct value is found
      if (map[idx] == cell_val) 
      {
        costmap.indexToCells(idx, nx, ny);
        costmap.mapToWorld(nx, ny, wx, wy); //Need everything in world coordinates

        E_hat_c = 0;
        //Info available from a cell decays with distance. 
        //Using sigmoid based on DARPA IROS 2022 papers. 
        //Basically we want to capture the uncertainty in information gain as the cell distance increases from a frontier
        //Subtract the information loss due to other robots positions 
        
        int iterator = 0;
        for (auto neighboring_robot_val : neighboring_robots_positions) 
        {
            costmap.mapToWorld(neighboring_robot_val.est_mx, neighboring_robot_val.est_my, rwx, rwy);
            // costmap.mapToWorld(neighboring_robot_val.true_mx, neighboring_robot_val.true_my, rwx, rwy);
            dist_j = sqrt(pow((rwx-wx),2) + pow((rwy-wy),2));
            
            // if(dist_j <= 2*sensor_range) //An overlap of a cell is only possible under this constraint.
            // if(dist_j <= sensor_range)
            // {
              // ROS_INFO("Distance to rel position (meters): %f", dist_j);
              // info_loss_with_distance = 1/(1+exp(neighboring_robot_val.omega*sigmoid_cost_steepness_*(dist_j-sigmoid_cost_midpoint_)));
              // E_hat_c += neighboring_robot_val.getTau() * info_loss_with_distance; //Check whether to include the loss due to a robot (e.g. only when its functional)
              
              // info_loss_with_distance = 1/(1+exp(sigmoid_cost_steepness_*(dist_j-sigmoid_cost_midpoint_-neighboring_robot_val.omega)));
              // info_loss_with_distance = 1/(1+exp(sigmoid_cost_steepness_*(dist_j-sigmoid_cost_midpoint_)));
              
              //NJ modified - new sigmoid function
              // info_loss_with_distance = sigmoid_cost_amplitude_ * 1/(1+exp((dist_j-sigmoid_cost_midpoint_)/sigmoid_cost_steepness_));
              info_loss_with_distance =  neighboring_robot_val.omega * 1/(1+exp((dist_j-sigmoid_cost_midpoint_)/sigmoid_cost_steepness_));
              // info_loss_with_distance = sigmoid_cost_amplitude_ * 1/(1+exp((dist_j-sigmoid_cost_midpoint_)/std::max(sigmoid_cost_steepness_,neighboring_robot_val.omega)));
              // info_loss_with_time = exp(-iterator)*info_loss_with_distance;
              // info_loss_with_time = neighboring_robot_val.omega*exp(-0.1*iterator)*info_loss_with_distance; //Scale the loss with convariance info also
              // info_loss_with_time = neighboring_robot_val.omega*info_loss_with_distance;s //Scale the loss with convariance info also
              // info_loss_with_time = neighboring_robot_val.omega*(exp(-iterator))*info_loss_with_distance; //We might want to sort the position estimates in the order of their covariance values.
              // info_loss_with_time = (exp(0.01*iterator)-0.95)*info_loss_with_distance;
              // ROS_INFO("Loss with distance: %f, Loss with time: %f", info_loss_with_distance, info_loss_with_time);
              // E_hat_c += neighboring_robot_val.getTau() * info_loss_with_time; //Check whether to include the loss due to a robot (e.g. only when its functional)
              E_hat_c += neighboring_robot_val.getTau() * info_loss_with_distance; //Check whether to include the loss due to a robot (e.g. only when its functional)
              iterator+=1;
            // }
        }
        // ROS_INFO("--------------------------------------------------");

        
        // if(neighboring_robots_positions.size()>0) 
        // {
        //   E_hat_c /= int(neighboring_robots_positions.size()); //Average out the loss
        // }
        
        dist_i = sqrt(pow((swx-wx),2) + pow((swy-wy),2));
        
        //Info from a cell cannot be negative. Do not consider negative values. The lowest info from a frontier should be 0 and not negative.
        result += std::max(0.0, double((sigmoid_cost_amplitude_ * 1/(1+exp((dist_i-sigmoid_cost_midpoint_)/sigmoid_cost_steepness_)) - E_hat_c))); // Use this when sensor bounding overlapp within sensor_range only
        // result += (sigmoid_cost_amplitude_ * 1/(1+exp((dist_i-sigmoid_cost_midpoint_)/sigmoid_cost_steepness_)) - E_hat_c); // Use this when sensor bounding overlapp within sensor_range only
        
        //result += 1/(1+exp(sigmoid_cost_steepness_*(dist_i-sigmoid_cost_midpoint_))) - E_hat_c; // Use this when sensor bounding overlapp within sensor_range only
        // result += std::max(float(0.0), 1/(1+exp(sigmoid_cost_steepness_*(dist_i-sigmoid_cost_midpoint_))) - E_hat_c); //Not using negative info gain for a cell 
        info_loss_at_a_frontier+=E_hat_c; // This will be higher than result variable.
        total_info_from_a_frontier+= (sigmoid_cost_amplitude_ * 1/(1+exp((dist_i-sigmoid_cost_midpoint_)/sigmoid_cost_steepness_)));
      }

      // iterate over all adjacent unvisited cells which are withing range from start cell (sx, sy)
      for (unsigned nbr : nhood8(idx, costmap)) 
      {
        if (!visited_flag[nbr]) 
        {
          costmap.indexToCells(nbr, nx, ny);
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

    // result = 100*total_info_loss_at_a_frontier/total_info_from_a_frontier;
    info_used_at_frontier_percent = 100*info_loss_at_a_frontier/total_info_from_a_frontier;

    return true;
  }


}

#endif
