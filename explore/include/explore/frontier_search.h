#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>
#include <unordered_map>
#include "Eigen/Eigen"
#include <explore/quadmap.h>

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double centroid_distance;
  double cost;
  double information_gain;
  geometry_msgs::Point initial;
  geometry_msgs::Point centroid;
  geometry_msgs::Point middle;
  geometry_msgs::Point furthest;
  std::vector<geometry_msgs::Point> points;
  int pos_id;
  int neighbors_count;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch()
  {
  }

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale, double min_frontier_size, double range, 
                 float alpha_parameter_, float beta_parameter_);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::Point position);
  std::vector<Frontier> searchFrontiers(geometry_msgs::Point& position);
  std::vector<Frontier> getMaxUtilityFrontiers(std::vector<Frontier>& frontier_list,
                                              quadmap::QuadMap& base_quadmap,
                                              int& robot_id);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);


  Frontier buildNewFrontier(unsigned int initial_cell,
                            unsigned int reference,
                            std::vector<bool>& frontier_flag,
                            std::vector<geometry_msgs::Point> rel_positions);


  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);
  
  /**
   * @brief computes frontier cost as information gain (expected number of unexplored cells) per unit of navigation effort.
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierUtility(const Frontier& frontier,
                        float& information_gain);
  
  double frontierUtility(const Frontier& frontier,
                        float& information_gain,
                        std::vector<geometry_msgs::Point> rel_positions,
                        float& effort);
  
  double coordinationCost(const Frontier& frontier,
                          std::vector<geometry_msgs::Point> rel_positions);

  float neighborhoodDistance(unsigned int start,
                            std::vector<geometry_msgs::Point> rel_positions);

  std::vector<Frontier> splitFrontier(const Frontier& frontier,
                                      unsigned int reference);

  std::vector<Frontier> splitFrontierPCA(const Frontier& frontier,
                                        unsigned int reference);

  void updateInfo(Frontier& frontier,
                  unsigned int reference);                                        

  private:
    costmap_2d::Costmap2D* costmap_;
    std::unordered_map<int, int> total_overlap_g_c_;
    unsigned char* map_;
    unsigned int size_x_;
    unsigned int size_y_;
    double potential_scale_; 
    double gain_scale_;
    double sensor_range_;
    double min_frontier_size_;
    float decay_rate_, eta_=1;
    int max_frontier_size_ = 0;
    float alpha_parameter_ = 1;
    float beta_parameter_ = 1;
};
}
#endif
