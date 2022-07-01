#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#define _USE_MATH_DEFINES

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <cmath>

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
 * @brief Find all the cells of some value within a range of some cell
 * @param result Count of such cells
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @param range Range to search within
 * @return True if a cell with the requested value was found
 */
bool nearestCellsWithinRange(int& result, unsigned int start, unsigned char val,
                            unsigned char val2, unsigned char val3, 
                            const costmap_2d::Costmap2D& costmap, double& range)
{
  const unsigned char* map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX(),
                     size_y = costmap.getSizeInCellsY();

  int free_cells =0, occupied_cells = 0;

  // ROS_INFO("Costmap resolution: %f", costmap.getResolution());
  // ROS_INFO("Range: %f", range);
  // ROS_INFO("range/resolution = %f ", range/costmap.getResolution());
  if (start >= size_x * size_y) {
    return false;
  }

  // initialize breadth first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  // push initial cell
  bfs.push(start);
  visited_flag[start] = true;
  unsigned int sx, sy, nx, ny;
  costmap.indexToCells(start, sx, sy);

  // search for neighbouring cell matching value
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // return if cell of correct value is found
    if (map[idx] == val) {
      result += 1;
    }
    else if (map[idx] == val2)
    {
      free_cells += 1;
    }
    else if(map[idx] == val3)
    {
      occupied_cells +=1;
    }

    // iterate over all adjacent unvisited cells
    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        costmap.indexToCells(nbr, nx, ny);
        float dist = sqrt(pow((sx-nx),2) + pow((sy-ny),2));
        if(dist*costmap.getResolution() <= range) 
        {
            bfs.push(nbr);
            // ROS_INFO("dist (map coord): %f", dist);
        }
        visited_flag[nbr] = true;
      }
    }
  }

  // ROS_INFO("Free = %d", free_cells);
  // ROS_INFO("Occupied = %d", occupied_cells);
  // ROS_INFO("Unknown = %d", result);
  // ROS_INFO("Total = %d", result + free_cells + occupied_cells);
  return true;
}

void informationGain(float& result, unsigned int start, unsigned char val,
                    const costmap_2d::Costmap2D& costmap, double& range,
                    std::unordered_map<int,int>& g_c, const float decay_rate,
                    std::vector<geometry_msgs::Point>& rel_pos_neighbors,
                    const float eta)
{
  const unsigned char* map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX(),
                     size_y = costmap.getSizeInCellsY();

  int free_cells =0, occupied_cells = 0, k_j=0;
  int rel_pos_size = rel_pos_neighbors.size();
  float dist_f=0, dist_j=0, a=0, beta=0, total_dist_j=0;

  // ROS_INFO("Costmap resolution: %f", costmap.getResolution());
  // ROS_INFO("Range: %f", range);
  // ROS_INFO("range/resolution = %f ", range/costmap.getResolution());

  // initialize breadth first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  // push initial cell
  bfs.push(start);
  visited_flag[start] = true;
  unsigned int sx, sy, nx, ny;
  double wx, wy;
  costmap.indexToCells(start, sx, sy);

  // search for neighbouring cell matching value
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // return if cell of correct value is found
    if (map[idx] == val) 
    {
      costmap.indexToCells(idx, nx, ny);
      costmap.mapToWorld(nx, ny, wx, wy);
      k_j = 0;
      for(int j=0; j<rel_pos_size; j++)
      {
        dist_j = sqrt(pow((rel_pos_neighbors[j].x-wx),2) + pow((rel_pos_neighbors[j].y-wy),2)); //Distance in world coordinates
        if(dist_j <= range) // the cell is also in the sensing radius of the neighboring robot j then it means there is overlap
        {
          k_j += 1;
        }
      }
      
      //Total calculated overlaps
      if(g_c.find(idx) == g_c.end())
      {
        g_c[idx] = k_j;
      }
      else
      {
        g_c[idx] += k_j;  
      }

      //Information value of the cell based on number of overlaps
      a = g_c[idx]>0 ? 0 : 1;  
      result += a + (1-a)*pow(M_E,(-decay_rate*g_c[idx])) ;
    }

    // iterate over all adjacent unvisited cells
    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        costmap.indexToCells(nbr, nx, ny);
        float dist_f = sqrt(pow((sx-nx),2) + pow((sy-ny),2));
        if(dist_f*costmap.getResolution() <= range) 
        {
            bfs.push(nbr);
            // ROS_INFO("dist (map coord): %f", dist);
        }
        visited_flag[nbr] = true;
      }
    }
  }

  // ROS_INFO("Free = %d", free_cells);
  // ROS_INFO("Occupied = %d", occupied_cells);
  // ROS_INFO("Unknown = %d", result);
  // ROS_INFO("Total = %d", result + free_cells + occupied_cells);

  costmap.mapToWorld(sx, sy, wx, wy); //World coordinates of the frontier centroid
  for(int j=0; j<rel_pos_size; j++)
  {
    total_dist_j += sqrt(pow((rel_pos_neighbors[j].x-wx),2) + pow((rel_pos_neighbors[j].y-wy),2));
  }

  beta = log10(eta*total_dist_j);
  // ROS_INFO("Total distance = %f", total_dist_j);
  // ROS_INFO("beta = %f", beta);
  result *= beta;
}

}
#endif
