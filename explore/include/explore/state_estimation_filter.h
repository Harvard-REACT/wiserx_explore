#ifndef WSR_EXPLORE_FILTER_
#define WSR_EXPLORE_FILTER_

#include <explore/utils.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace wsr_state_estimation
{
    class ExtendedKalmanFilter 
    {
    public:
        VectorXd x; // State vector [x, y, vx, vy]
        MatrixXd P; // Covariance matrix
        MatrixXd F; // State transition matrix
        MatrixXd Q; // Process noise covariance matrix
        MatrixXd R; // Measurement noise covariance matrix
        double dt; // Time step
        float clutter_intensity_;
        double gating_threshold_;
        std::vector<double> residual_error__ {0.0,0.0};
        std::vector<double> range_bearing__ {0.0,0.0};
        std::vector<VectorXd> residuals__;
        std::vector<float> likelihoods__;
        std::vector<float> angle_val__;
        std::vector<float> angle_pred__;
        std::vector<float> probs_vec__;
	    unsigned int mx__, my__, prev_mx__, prev_my__;

        ExtendedKalmanFilter(){}
        ~ExtendedKalmanFilter(){}
        ExtendedKalmanFilter(VectorXd x_val, double interval);
        VectorXd get_covariance();
        void predict();
        void update(const VectorXd &z, geometry_msgs::Pose &robot_i_position );
        void updatePDAF(float& range_measurement, 
                        std::vector<float>& bearing_measurements, 
                        geometry_msgs::Pose& robot_i_position) ;
        VectorXd h(const VectorXd &state, geometry_msgs::Pose &robot_i_position);
        MatrixXd calculateJacobian(const VectorXd &state, geometry_msgs::Pose& robot_i_position);
        MatrixXd calculateJacobianV2(const VectorXd &measurement);
        float MahalanobisDistance(VectorXd& measurement, MatrixXd& Covariance);
    
    };



    class Particle {
    public:
        Eigen::VectorXd state; // Particle state [x, y, vx, vy]
        double weight; // Particle weight

        Particle() : state(Eigen::VectorXd(4)), weight(1.0) {}
    };

    class ParticleFilter {
    public:
        std::vector<Particle> particles; // Set of particles
        int numParticles; // Number of particles
        std::default_random_engine gen; // Random engine for noise
        double dt; // Time step
        double std_pos__;
        double std_vel__;
        double velocity_x__;
        double velocity_y__;
        double std_range__ = 0.1 ; //meters
        double std_bearing__ = 0.09; //radians. ~ 5 degrees

        ParticleFilter(){}
        ~ParticleFilter(){}

        ParticleFilter(VectorXd x_val, double interval, double std_pos, double std_vel, double velocity_x, double velocity_y);
        
        void predict() ;

        void updateWeights(const VectorXd &z, geometry_msgs::Pose &robot_i_position) ;

        VectorXd getEstimate() ;

        MatrixXd computeCovariance(Eigen::VectorXd& estimate) ;

    };    
}




#endif
