#include "kalman_positioning/ukf.hpp"
#include <iostream>
#include <map>

/**
 * STUDENT ASSIGNMENT: Unscented Kalman Filter Implementation
 * * This file contains placeholder implementations for the UKF class methods.
 * Students should implement each method according to the UKF algorithm.
 * * Reference: Wan, E. A., & Van Der Merwe, R. (2000). 
 * "The Unscented Kalman Filter for Nonlinear Estimation"
 */

// ============================================================================
// CONSTRUCTOR
// ============================================================================

/**
 * @brief Initialize the Unscented Kalman Filter
 * * STUDENT TODO:
 * 1. Initialize filter parameters (alpha, beta, kappa, lambda)
 * 2. Initialize state vector x_ with zeros
 * 3. Initialize state covariance matrix P_ 
 * 4. Set process noise covariance Q_
 * 5. Set measurement noise covariance R_
 * 6. Calculate sigma point weights for mean and covariance
 */
UKF::UKF(double process_noise_xy, double process_noise_theta,
        double measurement_noise_xy, int num_landmarks) {
    
    // State dimension [x, y, theta, vx, vy]
    nx_ = 5; 

    // 1. Initialize state vector x as [0, 0, 0, 0, 0]T
    x_ = Eigen::VectorXd::Zero(nx_);

    // 2. Initialize covariance matrix P as identity matrix
    P_ = Eigen::MatrixXd::Identity(nx_, nx_);

    // 3. Set Process Noise Q = diag(xy, xy, theta, 0, 0)
    Q_ = Eigen::MatrixXd::Zero(nx_, nx_);
    Q_(0, 0) = process_noise_xy;
    Q_(1, 1) = process_noise_xy;
    Q_(2, 2) = process_noise_theta;
    Q_(3, 3) = 0.001; // Small velocity noise to keep matrix positive definite
    Q_(4, 4) = 0.001;

    // 4. Set Measurement Noise R = diag(xy, xy)
    R_ = Eigen::MatrixXd::Identity(2, 2) * measurement_noise_xy;

    // 5. Calculate UKF parameters: lambda and gamma
    // Standard spreading parameters for UKF
    double alpha = 0.001; 
    double kappa = 0.0;
    double beta = 2.0;
    
    lambda_ = alpha * alpha * (nx_ + kappa) - nx_;
    gamma_ = std::sqrt(nx_ + lambda_);

    // 6. Calculate weights for mean (Wm) and covariance (Wc)
    Wm_.resize(2 * nx_ + 1);
    Wc_.resize(2 * nx_ + 1);
    
    // Weight for the center point
    Wm_[0] = lambda_ / (nx_ + lambda_);
    Wc_[0] = Wm_[0] + (1.0 - alpha * alpha + beta);
    
    // Weights for the surrounding sigma points
    double common_weight = 1.0 / (2.0 * (nx_ + lambda_));
    for (int i = 1; i < 2 * nx_ + 1; i++) {
        Wm_[i] = common_weight;
        Wc_[i] = common_weight;
    }

    // Suppress unused parameter warning for num_landmarks
    (void)num_landmarks;
}/* */

// ============================================================================
// SIGMA POINT GENERATION
// ============================================================================

/**
 * @brief Generate sigma points from mean and covariance
 * * STUDENT TODO:
 * 1. Start with the mean as the first sigma point
 * 2. Compute Cholesky decomposition of covariance
 * 3. Generate 2*n symmetric sigma points around the mean
 */
std::vector<Eigen::VectorXd> UKF::generateSigmaPoints(const Eigen::VectorXd& mean,
                                                       const Eigen::MatrixXd& cosv) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================


    int n = nx_; 
    
    // Compute the Cholesky decomposition (square root) of the covariance matrix
    // This gives us the spread needed for the sigma points
    Eigen::MatrixXd L = cosv.llt().matrixL();

    // Create a temporary matrix to hold the points before converting to vector
    Eigen::MatrixXd sigma_mat = Eigen::MatrixXd(n, 2 * n + 1);

    // 1. The first point is simply the current mean
    sigma_mat.col(0) = mean;

    // 2. Generate the surrounding points in both directions (+ and -)
    for (int i = 0; i < n; i++) {
        sigma_mat.col(i + 1)     = mean + gamma_ * L.col(i);
        sigma_mat.col(i + 1 + n) = mean - gamma_ * L.col(i);
    }

    // Convert the Eigen Matrix columns into a std::vector for easier handling later
    std::vector<Eigen::VectorXd> sigma_vec;
    for (int i = 0; i < 2 * n + 1; ++i) {
        sigma_vec.push_back(sigma_mat.col(i));
    }

    return sigma_vec;
}

// ============================================================================
// PROCESS MODEL
// ============================================================================

/**
 * @brief Apply motion model to a state vector
 * * STUDENT TODO:
 * 1. Updates position: x' = x + dx, y' = y + dy
 * 2. Updates orientation: theta' = theta + dtheta (normalized)
 * 3. Updates velocities: vx' = dx/dt, vy' = dy/dt
 */
Eigen::VectorXd UKF::processModel(const Eigen::VectorXd& state, double dt,
                                  double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================

    // STUDENT IMPLEMENTATION STARTS HERE

    // 1. Create a vector for the predicted state
    Eigen::VectorXd x_out = state;

    // 2. Update position
    x_out(0) += dx; // Update x coordinate
    x_out(1) += dy; // Update y coordinate

    // 3. Update orientation and normalize it
    x_out(2) += dtheta; // Update theta
    // Normalize angle to [-pi, pi] using a while loop or atan2 logic
    while (x_out(2) > M_PI)  x_out(2) -= 2.0 * M_PI;
    while (x_out(2) < -M_PI) x_out(2) += 2.0 * M_PI;

    // 4. Calculate velocities
    // Avoid division by zero if dt is extremely small
    if (dt > 1e-6) {
        x_out(3) = dx / dt; // Velocity vx
        x_out(4) = dy / dt; // Velocity vy
    } else {
        x_out(3) = 0.0;
        x_out(4) = 0.0;
    }

    return x_out;

}

// ============================================================================
// MEASUREMENT MODEL
// ============================================================================

/**
 * @brief Predict measurement given current state and landmark
 * * STUDENT TODO:
 * 1. Calculate relative position: landmark - robot position
 * 2. Transform to robot frame using robot orientation
 * 3. Return relative position in robot frame
 */
Eigen::Vector2d UKF::measurementModel(const Eigen::VectorXd& state, int landmark_id) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    if (landmarks_.find(landmark_id) == landmarks_.end()) {
        return Eigen::Vector2d::Zero();
    }
    
    // Retrieve the true global position of the landmark from the map
    std::pair<double, double> lm = landmarks_.at(landmark_id);
    double lx = lm.first;
    double ly = lm.second;

    // Extract the robot's current estimated position and orientation
    double rx = state(0);
    double ry = state(1);
    double theta = state(2);

    // Calculate the difference between the landmark and the robot in the global frame
    double dx = lx - rx;
    double dy = ly - ry;

    // Transform this global difference into the robot's local body frame
    // This requires rotating the vector by the robot's orientation angle theta
    // The rotation matrix allows us to predict where the sensor should see the landmark
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    Eigen::Vector2d z_predicted;
    z_predicted(0) = cos_t * dx + sin_t * dy;  // Relative X in robot frame
    z_predicted(1) = -sin_t * dx + cos_t * dy; // Relative Y in robot frame

    return z_predicted;
}

// ============================================================================
// ANGLE NORMALIZATION
// ============================================================================

double UKF::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ============================================================================
// PREDICTION STEP
// ============================================================================

/**
 * @brief Kalman Filter Prediction Step (Time Update)
 * * STUDENT TODO:
 * 1. Generate sigma points from current state and covariance
 * 2. Propagate each sigma point through motion model
 * 3. Calculate mean and covariance of predicted sigma points
 * 4. Add process noise
 * 5. Update state and covariance estimates
 */
void UKF::predict(double dt, double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    // First, generate the sigma points based on the current state estimate and covariance
    // These points represent the probability distribution of where the robot might be
    std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);

    // Propagate each sigma point through the nonlinear process model
    // This simulates how the robot moves for each possible state in our distribution
    std::vector<Eigen::VectorXd> predicted_sigmas;
    predicted_sigmas.resize(2 * nx_ + 1);

    for (size_t i = 0; i < sigma_points.size(); ++i) {
        predicted_sigmas[i] = processModel(sigma_points[i], dt, dx, dy, dtheta);
    }

    // Calculate the predicted state mean by taking the weighted sum of the transformed sigma points
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(nx_);
    for (size_t i = 0; i < predicted_sigmas.size(); ++i) {
        x_pred += Wm_[i] * predicted_sigmas[i];
    }

    // Calculate the predicted state covariance matrix
    // This involves computing the weighted outer product of the difference between 
    // each sigma point and the predicted mean
    Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(nx_, nx_);
    for (size_t i = 0; i < predicted_sigmas.size(); ++i) {
        // Calculate the difference between the sigma point and the mean
        Eigen::VectorXd diff = predicted_sigmas[i] - x_pred;
        
        // Normalize the angle difference to ensure it stays within valid bounds
        // This is crucial for correct covariance calculation in orientation
        diff(2) = normalizeAngle(diff(2));

        P_pred += Wc_[i] * (diff * diff.transpose());
    }

    // Add the process noise covariance to account for uncertainty in the motion model
    // This prevents the filter from becoming overconfident
    P_pred += Q_;

    // Update the filter's internal state and covariance with the new predictions
    x_ = x_pred;
    P_ = P_pred;
    
    // Final safety normalization of the orientation angle
    x_(2) = normalizeAngle(x_(2));
}

// ============================================================================
// UPDATE STEP
// ============================================================================

/**
 * @brief Kalman Filter Update Step (Measurement Update)
 * * STUDENT TODO:
 * 1. Generate sigma points
 * 2. Transform through measurement model
 * 3. Calculate predicted measurement mean
 * 4. Calculate measurement and cross-covariance
 * 5. Compute Kalman gain
 * 6. Update state with innovation
 * 7. Update covariance
 */
void UKF::update(const std::vector<std::tuple<int, double, double, double>>& landmark_observations) {
    if (landmark_observations.empty()) {
        return;
    }
    
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    int nz = 2; 
    
    // Iterate through all available landmark observations
    for (const auto& obs : landmark_observations) {
        
        int id = std::get<0>(obs);
        double l_x_obs = std::get<1>(obs);
        double l_y_obs = std::get<2>(obs);
        
        if (!hasLandmark(id)) {
            continue;
        }

        // 1. Generate sigma points (Burada n_ kullaniyoruz, bu dogru)
        std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);

        // 2. Transform sigma points into measurement space
        std::vector<Eigen::VectorXd> z_sigmas;
        z_sigmas.resize(2 * nx_ + 1); // n_ sinif degiskeni
        
        for (size_t i = 0; i < sigma_points.size(); ++i) {
            z_sigmas[i] = measurementModel(sigma_points[i], id);
        }

        // 3. Calculate mean predicted measurement
       
        Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(nz); 
        for (size_t i = 0; i < z_sigmas.size(); ++i) {
            z_pred += Wm_[i] * z_sigmas[i];
        }

        // 4. Calculate covariance matrices
        
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nz, nz);
        Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(nx_, nz); // n_ (state) ve nz (measurement)

        for (size_t i = 0; i < sigma_points.size(); ++i) {
            // State residual
            Eigen::VectorXd x_diff = sigma_points[i] - x_;
            x_diff(2) = normalizeAngle(x_diff(2)); 

            // Measurement residual
            Eigen::VectorXd z_diff = z_sigmas[i] - z_pred;
            
            // Accumulate weighted covariances
            S += Wc_[i] * (z_diff * z_diff.transpose());
            Pxz += Wc_[i] * (x_diff * z_diff.transpose());
        }

        // Add measurement noise
        S += R_;

        // 5. Compute Kalman Gain
        Eigen::MatrixXd K = Pxz * S.inverse();

        // 6. Innovation
        Eigen::VectorXd z_actual(2);
        z_actual << l_x_obs, l_y_obs;
        
        Eigen::VectorXd innovation = z_actual - z_pred;
        
        // 7. Update state and covariance
        x_ += K * innovation;
        P_ -= K * S * K.transpose();
        
        x_(2) = normalizeAngle(x_(2));
    }
}

// ============================================================================
// LANDMARK MANAGEMENT
// ============================================================================

void UKF::setLandmarks(const std::map<int, std::pair<double, double>>& landmarks) {
    landmarks_ = landmarks;
}

bool UKF::hasLandmark(int id) const {
    return landmarks_.find(id) != landmarks_.end();
}