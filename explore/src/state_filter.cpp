#include<explore/state_estimation_filter.h>

wsr_state_estimation::ParticleFilter::ParticleFilter(VectorXd x_val, double interval, double std_pos, double std_vel, double velocity_x, double velocity_y) : numParticles(100) 
{
    dt = interval;
    std_pos__ = std_pos;
    std_vel__ = std_vel;
    velocity_x__ = velocity_x;
    velocity_y__ = velocity_y;
    std::normal_distribution<double> dist_x(x_val(0), std_pos__);
    std::normal_distribution<double> dist_y(x_val(1), std_pos__);
    std::normal_distribution<double> dist_vx(0, std_vel__); 
    std::normal_distribution<double> dist_vy(0, std_vel__); 

    for (int i = 0; i < numParticles; ++i) 
    {
        Particle particle;
        particle.state << dist_x(gen), dist_y(gen), dist_vx(gen), dist_vy(gen);
        particles.push_back(particle);
    }
}

void wsr_state_estimation::ParticleFilter::predict() 
{
    std::normal_distribution<double> dist_x(0, std_pos__);
    std::normal_distribution<double> dist_y(0, std_pos__);
    std::normal_distribution<double> dist_vx(0, std_vel__);
    std::normal_distribution<double> dist_vy(0, std_vel__);

    for (auto& particle : particles) 
    {
        particle.state[0] += velocity_x__ * dt + dist_x(gen);
        particle.state[1] += velocity_y__ * dt + dist_y(gen);
        particle.state[2] = velocity_x__ + dist_vx(gen);
        particle.state[3] = velocity_y__ + dist_vy(gen);
    }
}

void wsr_state_estimation::ParticleFilter::updateWeights(const VectorXd &measurements, geometry_msgs::Pose &robot_i_position) 
{
    double totalWeight = 0.0;
    
    for (auto& particle : particles) 
    {
        double x = particle.state[0] - robot_i_position.position.x;
        double y = particle.state[1] - robot_i_position.position.y;

        // Convert state from Cartesian to polar coordinates
        double particle_range = sqrt(x * x + y * y);
        double particle_bearing = atan2(y, x);

        // Calculate weight using Gaussian probability density function
        double range_diff = measurements(0) - particle_range;
        double bearing_diff = measurements(1) - particle_bearing;

        // Normalize bearing difference to be within [-pi, pi]
        bearing_diff = atan2(sin(bearing_diff), cos(bearing_diff));

        double weight_range = (1 / (sqrt(2 * M_PI) * std_range__)) * exp(-(range_diff * range_diff) / (2 * std_range__ * std_range__));
        double weight_bearing = (1 / (sqrt(2 * M_PI) * std_bearing__)) * exp(-(bearing_diff * bearing_diff) / (2 * std_bearing__ * std_bearing__));

        particle.weight = weight_range * weight_bearing;
        totalWeight += particle.weight;
    }

    // Normalize weights
    for (auto& particle : particles) 
    {
        particle.weight /= totalWeight;
    }
}

VectorXd wsr_state_estimation::ParticleFilter::getEstimate() 
{
    VectorXd estimate(4);
    estimate << 0, 0, 0, 0;

    for (const auto& particle : particles) 
    {
        estimate += particle.weight * particle.state;
    }

    std::cout << "Estx " << estimate(0) << std::endl;
    std::cout << "Esty " << estimate(1) << std::endl;

    return estimate;
}

MatrixXd wsr_state_estimation::ParticleFilter::computeCovariance(VectorXd &estimate) 
{
    MatrixXd covariance = MatrixXd::Zero(4, 4); // 4x4 matrix for [x, y, vx, vy]
    
    // Compute the weighted outer product of the deviations
    for (const auto& particle : particles)
     {
        VectorXd deviation = particle.state - estimate;
        covariance += particle.weight * (deviation * deviation.transpose());
    }

    return covariance;
}

wsr_state_estimation::ExtendedKalmanFilter::ExtendedKalmanFilter(VectorXd x_val, double interval): x(VectorXd(4)), P(MatrixXd(4, 4)), Q(MatrixXd(4, 4)), 
                                                                                                    R(MatrixXd(2, 2)), F(MatrixXd(4, 4))
{
    x = x_val;
    dt = interval;

    F << 1, 0, dt, 0, // State transition model
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // P << 1, 0, 0, 0, // Initial state covariance
    //     0, 1, 0, 0,
    //     0, 0, 1000, 0,
    //     0, 0, 0, 1000;

    //Updated P on July 31 2025
    P << 1, 0, 0, 0, // Initial state covariance
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Q << 0.1, 0, 0, 0, // Process noise covariance
        0, 0.1, 0, 0,
        0, 0, 0.1, 0,
        0, 0, 0, 0.1;

    // R << 0.1, 0,   // Measurement noise covariance (range (m), bearing (radians))
    //     0, 0.08;  // 30 cm and 16 degree of standard deviation for range and bearing


    //Updated R on July 31 2025
    R << 0.00001, 0,   // Measurement noise covariance (range (m), bearing (radians)). Ignore the impact of range.
        0, 0.01;  // 5 degree of standard deviation for range and bearing leading to 0.01 cov_x and cov_y

    // R << 0.01, 0,   // Measurement noise covariance (range (m), bearing (radians))
    //     0, 0.01;  // 10 cm and 5 degree of standard deviation for range and bearing leading to 0.01 cov_x and cov_y

    // R << 0.01, 0,   // Measurement noise covariance (range (m), bearing (radians))
    //     0, 0.001;  // 10cm and 1.81 degree of standard deviation for range and bearing 

    // R << 0.0025, 0,   // Measurement noise covariance (range (m), bearing (radians))
    //     0, 0.001;  // 5cm and 1.81 degree of standard deviation for range and bearing 


    // R << 0.001, 0,   // Measurement noise covariance (range (m), bearing (radians))
    //     0, 0.0001;  // 3cm and 0.5 degree of standard deviation for range and bearing 


}


// Predict the state and state covariance using the process model
void wsr_state_estimation::ExtendedKalmanFilter::predict() 
{
    x = F * x;
    P = F * P * F.transpose() + Q;
}

// Update the state by incorporating measurements
void wsr_state_estimation::ExtendedKalmanFilter::update(const VectorXd &z, geometry_msgs::Pose& robot_i_position ) 
{
    VectorXd z_pred = h(x, robot_i_position); // Predict measurement
    VectorXd y =  z - z_pred ; // Measurement residual
    
    range_bearing__.clear();
    range_bearing__.push_back(z_pred(0));
    range_bearing__.push_back(z_pred(1));

    residual_error__.clear();
    residual_error__.push_back(y(0));
    residual_error__.push_back(y(1));
    // std::cout << "Filter: Residial Range: "<< y(0) << " Bearing : " << y(1) << std::endl;
    
    MatrixXd H = calculateJacobian(x,robot_i_position); // Calculate Jacobian of the measurement model
    // MatrixXd H = calculateJacobianV2(z_pred); // Calculate Jacobian of the measurement model
    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain

    // Update state and covariance
    x = x + K * y;
    int size = x.size();
    MatrixXd I = MatrixXd::Identity(size, size);
    P = (I - K * H) * P;
}


// Update the state by incorporating measurements
void wsr_state_estimation::ExtendedKalmanFilter::updatePDAF(float& range_measurement, 
                                                            std::vector<float>& bearing_measurements, 
                                                            geometry_msgs::Pose& robot_i_position) 
{

    residuals__.clear();
    likelihoods__.clear();
    angle_val__.clear();
    angle_pred__.clear();
    probs_vec__.clear();
    residual_error__.clear();
    range_bearing__.clear();
    VectorXd y_bar = Eigen::Vector2d::Zero();
    MatrixXd P_update = 0*P;


    MatrixXd H = calculateJacobian(x,robot_i_position); // Calculate Jacobian of the measurement model
    MatrixXd S = H * P * H.transpose() + R;
    VectorXd z_pred = h(x, robot_i_position); // Predict measurement

    for(auto bval : bearing_measurements){
        VectorXd z(2); z << range_measurement, bval;        
        VectorXd y =  z - z_pred ; // Measurement residual
        y(1) = wrapToPi(y(1)); 
        float d2 = MahalanobisDistance(y, S);
        float gaussian_pdf = exp(-0.5*d2) / (2*M_PI*sqrt(S.determinant())) ;
        residuals__.push_back(y);
        likelihoods__.push_back(gaussian_pdf);
        angle_val__.push_back(z[1]);
        angle_pred__.push_back(z_pred[1]);
    }
    
    //Normalize the likelihoods
    //Clutter likelihood ==> Can we use the profile variance to somehow estimate this?
    float total = std::accumulate(likelihoods__.begin(), likelihoods__.end(), 0);
    if(total >0){
        for(auto prob: likelihoods__) probs_vec__.push_back(prob/total);
    }
    
    //Expected residual in measurement
    for(int k=0; k<int(probs_vec__.size()); k++){
        ROS_INFO("Angle_pred (deg): %f, Angle_measured (deg): %f, residuals: %f, probs: %f",
        angle_pred__[k]*180/3.14, angle_val__[k]*180/3.14, residuals__[k][1], probs_vec__[k]);
        y_bar += probs_vec__[k] * residuals__[k];
    }

    // Update state and covariance
    MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain
    x = x + K * y_bar;
    for(int i=0; i<probs_vec__.size(); i++) {
        MatrixXd outer_product = (residuals__[i]-y_bar) * (residuals__[i]-y_bar).transpose();
        P_update += probs_vec__[i] * (K * outer_product * K.transpose());
    }
    
    MatrixXd I = MatrixXd::Identity(P.rows(), P.cols());
    P = (I - K * H) * P * (I - K * H).transpose() + 
        K * (R+P_update) * K.transpose(); 

    range_bearing__.push_back(z_pred(0));
    range_bearing__.push_back(z_pred(1)*180.0 / M_PI);
    residual_error__.push_back(y_bar(0));
    residual_error__.push_back(y_bar(1)*180.0 / M_PI);
    // std::cout << "Filter: Residial Range: "<< y(0) << " Bearing : " << y(1) << std::endl;
    
}

float wsr_state_estimation::ExtendedKalmanFilter::MahalanobisDistance(VectorXd& measurement, MatrixXd& Covariance){
    return measurement.transpose() * Covariance.inverse() * measurement;
}

// Non-linear measurement model h(x)
VectorXd wsr_state_estimation::ExtendedKalmanFilter::h(const VectorXd &state, geometry_msgs::Pose& robot_i_position) 
{
    VectorXd measurement(2);
    double px = state(0) - robot_i_position.position.x;
    double py = state(1) - robot_i_position.position.y;
    measurement(0) = sqrt(px * px + py * py); // Range (meters)
    measurement(1) = atan2(py, px); // Bearing (radians)
    return measurement;
}

// Calculate the Jacobian matrix of the measurement model
MatrixXd wsr_state_estimation::ExtendedKalmanFilter::calculateJacobian(const VectorXd &state, geometry_msgs::Pose& robot_i_position) 
{
    MatrixXd Hj(2, 4);
    // double px = state(0);
    // double py = state(1);

    double dx = state(0) - robot_i_position.position.x;
    double dy = state(1) - robot_i_position.position.y;

    // Compute the Jacobian matrix
    double d = dx * dx + dy * dy;
    double sqrt_d = sqrt(d);
    
    // Check if division by zero might occur
    if (std::abs(d) < 0.0001) {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj; // Early return with unmodified Hj, which might be incorrect
    }

    // Recompute the Jacobian matrix with proper values
    Hj << (dx / sqrt_d), (dy / sqrt_d), 0, 0,
            -(dy / d), (dx / d), 0, 0;

    return Hj;
}


// Calculate the Jacobian matrix of the measurement model
MatrixXd wsr_state_estimation::ExtendedKalmanFilter::calculateJacobianV2(const VectorXd &measurement) 
{
    MatrixXd Hj(2, 4);
    
    // Recompute the Jacobian matrix with proper values
    //Reference : https://www.cs.cmu.edu/~16385/s17/Slides/16.4_Extended_Kalman_Filter.pdf
    Hj << cos(measurement(1)), sin(measurement(1)), 0, 0,
        -(sin(measurement(1))/measurement(0)), (cos(measurement(1))/measurement(0)), 0, 0;

    return Hj;
}




// int main() {
//     ExtendedKalmanFilter ekf;

//     // Simulate a noisy range and bearing measurement
//     // For example, a target at (1,1) with some measurement noise
//     double range_measurement = sqrt(2.0) + 0.1; // Some noise added
//     double bearing_measurement = atan2(1.0, 1.0) + 0.01; // Some noise added
//     VectorXd z(2);
//     z << range_measurement, bearing_measurement;

//     // Run prediction
//     ekf.predict();

//     // Update EKF with the noisy measurements
//     ekf.update(z);

//     // Print the updated state
//     std::cout << "Updated state x:\n" << ekf.x << std::endl;

//     return 0;
// }