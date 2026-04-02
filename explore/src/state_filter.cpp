#include<explore/state_estimation_filter.h>
#include<explore/custom_logger.h>

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

    CUSTOM_LOG_INFO("Estx %f", estimate(0));
    CUSTOM_LOG_INFO("Esty %f", estimate(1));

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
    mx__ = 0;
    my__ = 0;
    prev_mx__ = 0;
    prev_my__ = 0;

    F << 1, 0, dt, 0, // State transition model
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Initial state covariance. A very large initial velocity uncertainty can cause
    // the filter to become too uncertain, accepting all measurements as equally likely.
    // Since our velocity is 0.1 m/s, we can set the initial velocity uncertainty to a smaller value to reflect our confidence in the initial velocity estimate.
    P << 1, 0, 0, 0, // Position variance: std dev = 1m
        0, 1, 0, 0,
        0, 0, 0.01, 0, // Velocity variance: std dev = 0.1 m/s
        0, 0, 0, 0.01;

    // Process noise covariance Q. This models the uncertainty in the constant velocity model.
    // A robot's velocity is not truly constant; it accelerates and turns.
    // The max change in velocity in one step is ~0.2 m/s (e.g., reversing from 0.1 to -0.1).
    // Using a 3-sigma rule, the velocity variance is (0.2/3)^2 ≈ 0.0045.
    // For position, a max random drift of 30cm gives a variance of (0.30/3)^2 = 0.01.
    // A smaller Q for velocity makes the velocity estimate more stable.
    Q << 0.01, 0, 0, 0,
        0, 0.01, 0, 0,
        0, 0, 0.0045, 0,
        0, 0, 0, 0.0045;

    // //Updated R on July 31 2025 - Trial 4
    // R << 0.0001, 0,   // Measurement noise covariance (range (m), bearing (radians)). Ignore the impact of range.
    //     0, 0.2;  // 10 degree of standard deviation for range and bearing leading to 0.01 cov_x and cov_y

    // Trial 5
    // R << 0.1, 0,   // Measurement noise covariance (range (m), bearing (radians)). Ignore the impact of range.
    //     0, 0.2;  // 15 degree of standard deviation for range and bearing leading to 0.01 cov_x and cov_y

    //Trial 6
    R << 0.15, 0,   // Measurement noise covariance (range (m), bearing (radians)). Ignore the impact of range.
        0, 0.1;     // 5 degree of standard deviation for range and bearing leading to 0.01 cov_x and cov_y

    // Effective clutter intensity (lambda_c). This term is used to calculate
    // the probability that none of the measurements are from the target (beta_0).
    // A higher value means a higher chance of clutter.
    // It can be defined as: lambda * (1 - P_D*P_G)/P_D, where lambda is clutter
    // density, P_D is detection prob, P_G is gating prob.
    clutter_intensity_ = 1e-4;

    // Gating threshold from Chi-squared distribution.
    // For 2 degrees of freedom (range, bearing):
    // 90% confidence: 4.605
    // 95% confidence: 5.991
    // 99% confidence: 9.210
    gating_threshold_ = 5.991; // 95% confidence

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
    // CUSTOM_LOG_INFO("Filter: Residial Range: %f Bearing : %f", y(0), y(1));
    
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
    MatrixXd P_update = 0*R;


    MatrixXd H = calculateJacobian(x,robot_i_position); // Calculate Jacobian of the measurement model
    MatrixXd S = H * P * H.transpose() + R;
    VectorXd z_pred = h(x, robot_i_position); // Predict measurement
    CUSTOM_LOG_INFO("Obtained bearing measurements size %d", int(bearing_measurements.size()));
    for(auto bval : bearing_measurements){
        VectorXd z(2); 
        z << range_measurement, bval;        
        VectorXd y =  z - z_pred ; // Measurement residual
        y(1) = wrapToPi(y(1)); 
        float d2 = MahalanobisDistance(y, S);
        CUSTOM_LOG_INFO("Angle_measured (deg): %f, d2: %f",z[1]*180/3.14, d2);
        if(d2 < gating_threshold_){
	      float gaussian_pdf = exp(-0.5*d2) / (2*M_PI*sqrt(S.determinant())) ;
          residuals__.push_back(y);
          likelihoods__.push_back(gaussian_pdf);
          angle_val__.push_back(z[1]);
          angle_pred__.push_back(z_pred[1]);
	}
    }
    
    //Normalize the likelihoods
    float total_likelihood = 0;
    for(auto val: likelihoods__) total_likelihood += val;

    // Clutter likelihood. This represents the probability of false alarms or missed detections.
    float normalization_factor = total_likelihood + clutter_intensity_;
    float beta_0 = clutter_intensity_ / normalization_factor;
    
    //Only update if there are measurements
    if(total_likelihood > 0){
        for(auto prob: likelihoods__) probs_vec__.push_back(prob/normalization_factor);
   
       //Expected residual in measurement
       double sin_sum = 0, cos_sum = 0;
        for(int k=0; k<int(probs_vec__.size()); k++){
            CUSTOM_LOG_INFO("Angle_pred (deg): %f, Angle_measured (deg): %f, residuals: %f, probs: %f",
            angle_pred__[k]*180/3.14, angle_val__[k]*180/3.14, residuals__[k][1], probs_vec__[k]);
            y_bar(0) += probs_vec__[k] * residuals__[k](0); // Range is linear
            sin_sum += probs_vec__[k] * std::sin(residuals__[k](1));
            cos_sum += probs_vec__[k] * std::cos(residuals__[k](1));
        }
        y_bar(1) = std::atan2(sin_sum, cos_sum);

        // Update state and covariance
        MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain
        x = x + K * y_bar;

        // Correctly calculate the innovation spread covariance P_update
        // P_update = (Σ [βᵢ * yᵢ * yᵢᵀ]) - (ȳ * ȳᵀ)
        MatrixXd P_update_sum_term = Eigen::MatrixXd::Zero(2, 2);
        for(int i=0; i<probs_vec__.size(); i++) {
            MatrixXd outer_product = residuals__[i] * residuals__[i].transpose();
            P_update_sum_term += probs_vec__[i] * outer_product;
        }
        P_update = P_update_sum_term - (y_bar * y_bar.transpose());

        // Full PDAF covariance update
        MatrixXd I = MatrixXd::Identity(P.rows(), P.cols());
        MatrixXd P_c = (I - K * H) * P;
        MatrixXd P_tilde = K * P_update * K.transpose();
        P = beta_0 * P + (1 - beta_0) * P_c + P_tilde;
        range_bearing__.push_back(z_pred(0));
        range_bearing__.push_back(z_pred(1)*180.0 / M_PI);
        residual_error__.push_back(y_bar(0));
        residual_error__.push_back(y_bar(1)*180.0 / M_PI);
        // CUSTOM_LOG_INFO("Filter: Residial Range: %f Bearing : %f", y_bar(0), y_bar(1));
    }
    else{
        CUSTOM_LOG_INFO("No valid measurements");
    }
    
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
    MatrixXd Hj = MatrixXd::Zero(2, 4);
    double dx = state(0) - robot_i_position.position.x;
    double dy = state(1) - robot_i_position.position.y;

    // Compute the Jacobian matrix
    double d = dx * dx + dy * dy;
    double sqrt_d = sqrt(d);
    
    // Check if division by zero might occur
    if (std::abs(d) < 0.0001) {
        CUSTOM_LOG_ERROR("CalculateJacobian () - Error - Division by Zero");
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