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
   * @return True if a cell with the requested value was found
   */
  bool InfoNearestCellsWithinRange(float& result, unsigned int start, unsigned char cell_val,
                                   const costmap_2d::Costmap2D& costmap, double& range,
                                   std::vector<quadmap::Node>& neighboring_robots_positions)
  {
    const unsigned char* map = costmap.getCharMap();
    const unsigned int size_x = costmap.getSizeInCellsX(),
                      size_y = costmap.getSizeInCellsY();

    float dist_i = 0;
    float dist_j = 0;
    int sigmoid_cost_midpoint_ = range/2;
    int sigmoid_cost_steepness_ = 2;
    float E_hat_c = 0;
    float info_loss = 0;
    unsigned int sx, sy, nx, ny;
    double swx, swy, wx, wy, rwx, rwy;

    if (start >= size_x * size_y) {
      return false;
    }

    // initialize breadth first search
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(size_x * size_y, false);

    // push initial cell
    bfs.push(start);
    visited_flag[start] = true;
    costmap.indexToCells(start, sx, sy);
    costmap.mapToWorld(sx, sy, swx, swy);

    // search for neighbouring cell matching value
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
        info_loss = 0;

        //Info available from a cell decays with distance. 
        //Using sigmoid based on DARPA IROS 2022 papers. 
        //Basically we want to capture the uncertainty in information gain as the cell distance increases from a frontier
        //Subtract the information loss due to other robots positions        
        for (auto neighboring_robot_val : neighboring_robots_positions) 
        {
            costmap.mapToWorld(neighboring_robot_val.est_x, neighboring_robot_val.est_y, rwx, rwy);
            dist_j = sqrt(pow((rwx-wx),2) + pow((rwy-wy),2));
            info_loss = 1/(1+exp(neighboring_robot_val.omega*sigmoid_cost_steepness_*(dist_j-sigmoid_cost_midpoint_)));
            E_hat_c += neighboring_robot_val.getTau() * info_loss; //Check whether to include the loss due to a robot (e.g. only when its functional)
        }

        if(neighboring_robots_positions.size()>0) 
        {
          E_hat_c /= int(neighboring_robots_positions.size()); //Average out the loss
        }
        
        dist_i = sqrt(pow((swx-wx),2) + pow((swy-wy),2));
        result += ( 1/(1+exp(sigmoid_cost_steepness_*(dist_i-sigmoid_cost_midpoint_))) - E_hat_c); 
      }

      // iterate over all adjacent unvisited cells which are withing range from start cell (sx, sy)
      for (unsigned nbr : nhood8(idx, costmap)) 
      {
        if (!visited_flag[nbr]) {
          costmap.indexToCells(nbr, nx, ny);
          costmap.mapToWorld(nx, ny, wx, wy);
          dist_i = sqrt(pow((swx-wx),2) + pow((swy-wy),2));
          if(dist_i <= range) 
          {
              bfs.push(nbr);
          }
          visited_flag[nbr] = true;
        }
      }
    }

    return true;
  }

}
#endif
