/*
'''
Original Python code: https://scipython.com/blog/quadtrees-2-implementation-in-python/
Modified by : Ninad Jadhav
'''
*/

#ifndef WSR_EXPLORE_QUADMAP_
#define WSR_EXPLORE_QUADMAP_


#include <iostream>
#include <cmath>
#include <string>
#include <queue>
#include <memory>
#include <vector>
#include <functional>
#include <ros/ros.h>

namespace quadmap
{
    class Node 
    {
        public:     
            int* robot_j_node_tau; //Denotes robot is funcitonal or not. We assume all robots to be functional so tau = 1
            int robot_id_copy;   
            float true_mx=0;
            float true_my=0;
            float est_mx=0;
            float est_my=0;
            Node* prev_node = NULL; // Might not be useful for now
            Node* next_node = NULL;
            
            double cov_x = 0;
            double cov_y = 0;
            float omega = 0;
            //TODO update this 
            float gamma_val = 1;
            int timestep = 0;
                        
            // Constructors
            //Note that map coordinates are in unsigned int, but here they are stored as float
            Node(float x, float y, int& robot_tau, int robot_id, int timestep): true_mx(x), true_my(y), robot_j_node_tau(&robot_tau), robot_id_copy(robot_id), timestep(timestep) 
            {
                est_mx = true_mx;
                est_my = true_my;
            }
            
            void add_position_noise(float noisy_map_x, float noisy_map_y)
            {
                est_mx = noisy_map_x;
                est_my = noisy_map_y;
            }

            // Member functions
            float distanceTo(const Node& other) const
            {
                // ROS_INFO("Function: est_mx, est_my : %f, %f", est_mx, est_my);
                // ROS_INFO("Function: Other x,y : %f, %f", other.true_mx, other.true_my);
                return std::hypot(est_mx - other.true_mx, est_my - other.true_my);
            }
            
            float truedistanceTo(const Node& other) const
            {
                return std::hypot(true_mx - other.true_mx, true_my - other.true_my);
            }

            void updateOmega(double cov_x, double cov_y) //Covariance is in world coordinates as are all distance measurements
            {
                omega = std::min(1.0,1/(cov_x+cov_y)); //Inverse of the Trace of the covariance matrix
                ROS_INFO("OMEGA = %f", omega);
            }

            int getTau() const 
            {
                return *robot_j_node_tau;
            }

            int getRobotID() const 
            {
                // std::cout << "Robot ID " <<robot_id_copy << std::endl;
                return robot_id_copy;
            }
    };


    class Robot
    {
        public:
            int robot_tau = 1; // denotes if a robot is functional or not.
            int robot_id = 0; 
            std::queue<Node> node_information;
    };



    class Rect {
        public:
            float cx, cy; // Center position of the rectangle
            float w, h; // Width and height of the rectangle
            float west_edge, east_edge, north_edge, south_edge; // Edges of the rectangle
            
            // Constructor initializes the rectangle with center (cx, cy), width (w), and height (h)
            // and calculates the edges based on these values.
            Rect()
            {
            }
            ~Rect()
            {
            }
            
            Rect(float cx, float cy, float w, float h) : cx(cx), cy(cy), w(w), h(h) 
            {
                west_edge = cx - w / 2;
                east_edge = cx + w / 2;
                north_edge = cy - h / 2;
                south_edge = cy + h / 2;
            }

            // Prints the edges of the rectangle to standard output.
            void printEdges() const 
            {
                std::cout << "(" << west_edge << ", " << north_edge << ", "
                        << east_edge << ", " << south_edge << ")" << std::endl;
            }

            // Checks if a given point (x, y) is within the rectangle.
            // Returns true if the point is inside; otherwise, false.
            bool containsTrue(const Node& point) const 
            {
                return point.true_mx >= west_edge && point.true_mx < east_edge &&
                    point.true_my >= north_edge && point.true_my < south_edge;
            }

            bool contains(const Node& point) const 
            {
                // ROS_INFO("---------------------------");
                // ROS_INFO("west_edge %f", west_edge);
                // ROS_INFO("east_edge %f", east_edge);
                // ROS_INFO("north_edge %f", north_edge);
                // ROS_INFO("south_edge %f", south_edge);

                return point.est_mx >= west_edge && point.est_mx < east_edge &&
                    point.est_my >= north_edge && point.est_my < south_edge;
            }

            // Determines if another rectangle intersects with this one.
            // Returns true if there is an intersection; otherwise, false.
            bool intersects(const Rect& other) const {
                return !(other.west_edge > east_edge ||
                        other.east_edge < west_edge ||
                        other.north_edge > south_edge ||
                        other.south_edge < north_edge);
            }

            // Placeholder for the drawing method.
            // To be implemented using a specific graphics library.
            void draw() const {
                // Implement drawing using a graphics library.
                std::cout << "Drawing rectangle from (" << west_edge << ", " << north_edge
                        << ") to (" << east_edge << ", " << south_edge << ")" << std::endl;
            }
    };



    class QuadMap 
    {
        public:
            Rect boundary;
            int max_points = 4;
            int depth;
            std::vector<Node> points;
            // std::vector<const Node*> points;
            // std::vector<std::reference_wrapper<const Node>> points;
            bool divided;
            std::unique_ptr<QuadMap> nw, ne, se, sw;
            float sensor_range=0;
            float sensor_range_map_res = 0;
            float map_resolution = 0;
            int quadmap_ID = 0;
            int filled_val = 0;
            int total_cells = 0;
        
        QuadMap()
        {
        }
        
        QuadMap(Rect boundary, float sensor_range = 10, float map_resolution=0.25, int depth = 0)
        : boundary(boundary), sensor_range(sensor_range), map_resolution(map_resolution), depth(depth), divided(false)
        {
            sensor_range_map_res = sensor_range/map_resolution;
            total_cells = int((boundary.w*boundary.h)/(sensor_range_map_res*sensor_range_map_res));

            if (boundary.w != boundary.h) 
            {
                std::cerr << "Error: Initialize with same dimensions of length and breadth" << std::endl;
            }
            // ROS_INFO("Quadmap width:%f , height:%f", boundary.w, boundary.h);
            // ROS_INFO("sensor_range %f ", sensor_range);
            // ROS_INFO("map_resolution %f ", map_resolution);
            // ROS_INFO("sensor_range_map_res %f ", sensor_range_map_res);
        }

        /**
         * Divides the current QuadMap node into four child nodes.
         *
         * This method is called when the number of points in the node exceeds
         * the maximum allowed points per node (max_points). It divides the current
         * node's region into four equal quadrants (northwest, northeast, southeast,
         * and southwest) and initializes a child QuadMap for each quadrant with
         * the new boundaries. This division allows the QuadMap to maintain a balanced
         * distribution of points across its nodes, enhancing spatial query performance.
         *
         * Each child QuadMap inherits the max_points limit and sensor range from
         * its parent but has its depth incremented by one to reflect its level in the tree.
         * The method marks the current node as divided to prevent further unnecessary divisions.
         */
        
        void update_quadmap_ID(int& val)
        {
            quadmap_ID = val;
        }
        
        int get_quadmap_ID()
        {
            return quadmap_ID;
        }

        void divide() 
        {
            float halfWidth = boundary.w / 2.0;
            float halfHeight = boundary.h / 2.0;
            float centerX = boundary.cx;
            float centerY = boundary.cy;

            // Define the boundaries for the four quadrants of the current region.
            Rect nwBoundary(centerX - halfWidth / 2, centerY - halfHeight / 2, halfWidth, halfHeight);
            Rect neBoundary(centerX + halfWidth / 2, centerY - halfHeight / 2, halfWidth, halfHeight);
            Rect seBoundary(centerX + halfWidth / 2, centerY + halfHeight / 2, halfWidth, halfHeight);
            Rect swBoundary(centerX - halfWidth / 2, centerY + halfHeight / 2, halfWidth, halfHeight);

            // Create child QuadMap nodes for each quadrant with the respective boundaries.
            nw = std::make_unique<QuadMap>(nwBoundary, sensor_range, map_resolution, depth + 1);
            ne = std::make_unique<QuadMap>(neBoundary, sensor_range, map_resolution, depth + 1);
            se = std::make_unique<QuadMap>(seBoundary, sensor_range, map_resolution, depth + 1);
            sw = std::make_unique<QuadMap>(swBoundary, sensor_range, map_resolution, depth + 1);

            divided = true; // Indicate that this node has now been divided into child nodes.
        }

            /**
             * Attempts to insert a point into the QuadMap, traversing all the way to the
             * last leaf node based on boundary conditions.
             *
             * This function checks if the current node's boundary width is less than twice
             * the sensor range, in which case it does not divide further and attempts to insert
             * the point into the current node if the point is within the node's boundary.
             *
             * If the boundary width equals twice the sensor range and the point lies within
             * the boundary, the point is added to this node. Otherwise, if the node's boundary
             * width is greater than twice the sensor range, the function attempts to divide the
             * node further and insert the point into the appropriate child node until the condition
             * for not dividing further is met.
             *
             * @param point The point to insert into the QuadMap.
             * @return True if the point is successfully inserted, False otherwise.
             */
            bool insert_till_end(Node& point) 
            {
                // if (boundary.w < 2*sensor_range_map_res) 
                // {
                //     // Do not divide further if boundary width is less than twice the sensor range.
                //     return false;
                // }

                // if (boundary.w == 2*sensor_range_map_res)
                // {
                //     if (boundary.contains(point)) 
                //     {
                //         // Add the point here if the boundary contains the point and meets the condition.
                //         points.push_back(std::ref(point));
                //         std::cout << "========== Successfully inserted a point in the map " << std::endl;
                //         return true;
                //     }
                //     return false;
                // }
                if (boundary.w <= sensor_range_map_res) 
                {
                    if (boundary.contains(point))
                    // if (boundary.containsTrue(point)) 
                    {
                        // Add the point here if the boundary contains the point and meets the condition.
                        // points.push_back(std::ref(point));
                        points.push_back(point);
                        // ROS_INFO("Boundary w,h : %f,%f", boundary.w, boundary.h);
                        // ROS_INFO("Boundary cx,xy : %f,%f", boundary.cx, boundary.cy);                         
                        // ROS_INFO("west_edge %f", boundary.west_edge);
                        // ROS_INFO("east_edge %f", boundary.east_edge);
                        // ROS_INFO("north_edge %f", boundary.north_edge);
                        // ROS_INFO("south_edge %f", boundary.south_edge);                        
                        // ROS_INFO("Successfully inserted a true point(x,y,ID) = %f,%f,%d ", point.est_mx, point.est_my, point.getRobotID());
                        // ROS_INFO("---------------------------");
                                               
                        // for (auto ptr : points)
                        // {
                        //     ROS_INFO("Insert Check inserted a true point(x,y,ID) = %f,%f,%d ", ptr.true_mx, ptr.true_my, ptr.getRobotID());
                        // }
                        
                        
                        // this->filled_val += 1;
                        
                        return true;
                    }
                    
                     // Do not divide further if boundary width is less than twice the sensor range.
                    return false;
                } 
                else 
                {
                    // Still need to divide further to go all the way to the last node.
                    if (!divided) {
                        divide(); // Divide the node if it has not been divided yet.
                    }

                    // Attempt to insert the point into one of the child nodes.
                    if (nw->insert_till_end(point)) return true;
                    if (ne->insert_till_end(point)) return true;
                    if (se->insert_till_end(point)) return true;
                    if (sw->insert_till_end(point)) return true;
                    
                    return false; // Return false if the point could not be inserted into any child nodes.
                }
            }


        /**
         * Finds the nodes within the quadtree that lie within a specified radius of a given centre node.
         * 
         * The search is optimized by first considering a bounding square around the circle defined by the radius.
         * This method checks if the quadtree node's boundary intersects with this square. If not, it concludes
         * there are no nodes of interest in this node. If there is an intersection, it further checks each node
         * within the node to determine if it lies within the specified radius from the centre. This method combines
         * both rectangular boundary checks and circular distance checks to efficiently filter out nodes.
         * 
         * @param boundary A Rect object representing the bounding square of the search circle.
         * @param centre The centre node of the search circle.
         * @param radius The radius of the search circle.
         * @return True if any nodes are found within the radius, False otherwise.
         */
        void query_filled(float& filled_cell_count)
        {
            if (boundary.w <= sensor_range_map_res) 
            {
                // Iterate through all the points for "fuctional robots" within the boundary and compute the filled_val on the fly
                this->filled_val = 0;
                for(auto point : this->points)
                {
                    this->filled_val += point.getTau(); // This will be 0 if a robot j becomes non-functional (dead and cannot get pings) during the middle of the exploration.
                    // ROS_INFO("Robot: %d, tau: %d\n", point.getRobotID(), point.getTau());
                }
                
                // ROS_INFO("hgrid cell filled val: %d\n", this->filled_val);
                if (this->filled_val > 0) //Atleast 1 position estimates inside it, since sometimes ekf will generate spurious measurements
                {
                    filled_cell_count += 1;
                }
            } 
            else 
            {
                // Still need to divide further to go all the way to the last node.
                if (!divided) {
                    divide(); // Divide the node if it has not been divided yet.
                }

                nw->query_filled(filled_cell_count);
                ne->query_filled(filled_cell_count);
                se->query_filled(filled_cell_count);
                sw->query_filled(filled_cell_count);
                
            }
        }


        /**
         * Finds the nodes within the quadtree that lie within a specified radius of a given centre node.
         * 
         * The search is optimized by first considering a bounding square around the circle defined by the radius.
         * This method checks if the quadtree node's boundary intersects with this square. If not, it concludes
         * there are no nodes of interest in this node. If there is an intersection, it further checks each node
         * within the node to determine if it lies within the specified radius from the centre. This method combines
         * both rectangular boundary checks and circular distance checks to efficiently filter out nodes.
         * 
         * @param boundary A Rect object representing the bounding square of the search circle.
         * @param centre The centre node of the search circle.
         * @param radius The radius of the search circle.
         * @param found_nodes A reference to a vector of Node objects where nodes found within the radius are added.
         * @return True if any nodes are found within the radius, False otherwise.
         */
        bool query_circle(const Rect& query_boundary, const Node& centre, float radius, std::vector<Node>& found_nodes)
        {
            if (!this->boundary.intersects(query_boundary)) 
            {
                // If the domain of this node does not intersect the search region, skip this node.
                return false;
            }

            bool found = false;

            // Search this node's nodes to see if they lie within the circular boundary. Exclude a robot's own positions.
            if(points.size()>0)
            {
                // ROS_INFO("---- Found an intersecting boundary---");
                // ROS_INFO("Boundary w,h : %f,%f", this->boundary.w, this->boundary.h);
                // ROS_INFO("Boundary cx,cy : %f,%f", this->boundary.cx, this->boundary.cy);
                // ROS_INFO("Points dim %d", int(points.size()));
            }

            int it_v = 0;
            for(auto point : points)
            {
                // const Node point = ref.get();

                // ROS_INFO(" %d) Quadmap boundary true point = %f,%f ",it_v++, point.true_mx, point.true_my);
                // ROS_INFO("Quadmap west_edge %f", this->boundary.west_edge);
                // ROS_INFO("Quadmap east_edge %f", this->boundary.east_edge);
                // ROS_INFO("Quadmap north_edge %f", this->boundary.north_edge);
                // ROS_INFO("Quadmap south_edge %f", this->boundary.south_edge); 

                auto a = query_boundary.contains(point);
                // auto a = query_boundary.containsTrue(point);
                // ROS_INFO("Query west_edge %f", query_boundary.west_edge);
                // ROS_INFO("Query east_edge %f", query_boundary.east_edge);
                // ROS_INFO("Query north_edge %f", query_boundary.north_edge);
                // ROS_INFO("Query south_edge %f", query_boundary.south_edge); 

                auto b = point.distanceTo(centre);
                // auto b = point.truedistanceTo(centre);
                int c = point.robot_id_copy;
                
                // ROS_INFO("Point ID = %d ", c);
                // ROS_INFO("Query Boundary contains point = %d", a);
                // ROS_INFO("Distance to center = %f", b);
                // ROS_INFO("Own ID: %d", centre.getRobotID());

                if (a && b<= radius && c!= centre.getRobotID()) {
                    // ROS_INFO("Success - Found node!!!");
                    found_nodes.push_back(point); //Store pointer to the node data point.
                    // ROS_INFO("Node size: %ld", found_nodes.size());
                    found = true;
                }
            }

            // If this node has been divided, recurse the search into the child nodes.
            // ROS_INFO("[query_circle before] Node size: %ld", found_nodes.size());

            if (divided) {
                found |= nw->query_circle(boundary, centre, radius, found_nodes);
                found |= ne->query_circle(boundary, centre, radius, found_nodes);
                found |= se->query_circle(boundary, centre, radius, found_nodes);
                found |= sw->query_circle(boundary, centre, radius, found_nodes);
            }

            // ROS_INFO("[query_circle after] Node size: %ld", found_nodes.size());
            return found;
        }

        /**
         * Finds nodes within the QuadMap that lie within a specified radius of a given center node.
         *
         * @param centre The center node of the search area.
         * @param radius The radius of the search area.
         * @param found_nodes Reference to a vector where found nodes will be added.
         * @return True if any nodes are found within the radius, False otherwise.
         */
        bool query_radius(const Node& centre, float radius, std::vector<Node>& found_nodes) 
        {
            // Calculate the bounding box for the search circle centered on robot i's frontier.
            float centerX = centre.true_mx;
            float centerY = centre.true_my;
            float rad = radius/map_resolution;
            
            // ROS_INFO("Rad : %f ", rad);
            Rect query_boundary(centerX, centerY, 2 * rad, 2 * rad);

            // Call query_circle with the calculated boundary, center, and radius.
            bool final_output = query_circle(query_boundary, centre, rad, found_nodes);

            // ROS_INFO("[query_radius] Node size: %ld", found_nodes.size());

            return final_output;

        }


        /**
         * Calculates the total number of points contained within this QuadMap node and all of its subdivisions.
         * 
         * This method recursively traverses the QuadMap, starting from this node and including all its child nodes if any,
         * summing up the number of points contained at each level. The process accounts for both the points directly stored
         * in the current node and those stored in its divided children, if the node has been divided.
         * 
         * The method ensures an accurate count of all points within the encompassed spatial region of the QuadMap,
         * providing a means to understand the density or distribution of points managed by the QuadMap structure.
         * 
         * @return The total count of points within this QuadMap node and its child nodes.
         */
        int tree_size() const 
        {
            // Initial count is the number of points in the current node.
            int npoints = points.size();

            // If the node has been divided, recursively add the count from each child.
            if (divided) 
            {
                npoints += nw->tree_size() + ne->tree_size() + se->tree_size() + sw->tree_size();
            }

            return npoints;
        }
        
    };
}

#endif
