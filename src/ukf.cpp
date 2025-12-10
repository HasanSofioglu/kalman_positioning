#include "kalman_positioning/ukf.hpp"
#include <iostream>
#include <map>

/**
 * STUDENT ASSIGNMENT: Unscented Kalman Filter Implementation
 * 
 * This file contains placeholder implementations for the UKF class methods.
 * Students should implement each method according to the UKF algorithm.
 * 
 * Reference: Wan, E. A., & Van Der Merwe, R. (2000). 
 * "The Unscented Kalman Filter for Nonlinear Estimation"
 */

// ============================================================================
// CONSTRUCTOR
// ============================================================================

/**
 * @brief Initialize the Unscented Kalman Filter
 * 
 * STUDENT TODO:
 * 1. Initialize filter parameters (alpha, beta, kappa, lambda)
 * 2. Initialize state vector x_ with zeros
 * 3. Initialize state covariance matrix P_ 
 * 4. Set process noise covariance Q_
 * 5. Set measurement noise covariance R_
 * 6. Calculate sigma point weights for mean and covariance
 */
UKF::UKF(double process_noise_xy, double process_noise_theta,
         double measurement_noise_xy, int num_landmarks)
    : nx_(5), nz_(2) {
    
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    

    // 1. Initialize filter parameters (alpha, beta, kappa, lambda)
    // Formula: lambda = alpha^2 * (n + kappa) - n
    lambda_ = ALPHA * ALPHA * (nx_ + KAPPA) - nx_;
    
    // gamma = sqrt(n + lambda) -> Scaling factor for sigma points
    gamma_ = std::sqrt(nx_ + lambda_);

    // 2. Initialize state vector x_ with zeros
    x_ = Eigen::VectorXd::Zero(nx_);

    // 3. Initialize state covariance matrix P_ as Identity
    // Identity matrix implies independent uncertainties initially
    P_ = Eigen::MatrixXd::Identity(nx_, nx_);

    // 4. Set process noise covariance Q_
    // Task Requirement: Q = diag(process_noise_xy, process_noise_xy, process_noise_theta, 0, 0)
    Q_ = Eigen::MatrixXd::Zero(nx_, nx_);
    Q_(0, 0) = process_noise_xy;    // Noise in X
    Q_(1, 1) = process_noise_xy;    // Noise in Y
    Q_(2, 2) = process_noise_theta; // Noise in Theta
    // Velocities (index 3 and 4) assume 0 process noise for this assignment

    // 5. Set measurement noise covariance R_
    // Task Requirement: R = diag(measurement_noise_xy, measurement_noise_xy)
    R_ = Eigen::MatrixXd::Identity(nz_, nz_) * measurement_noise_xy;

    // 6. Calculate sigma point weights for mean and covariance
    // Total sigma points = 2 * n + 1
    int num_sigmas = 2 * nx_ + 1;
    Wm_.resize(num_sigmas);
    Wc_.resize(num_sigmas);

    // -- Weight for the Mean (Center Point) --
    // Wm_0 = lambda / (n + lambda)
    Wm_[0] = lambda_ / (double)(nx_ + lambda_);
    
    // -- Weight for the Covariance (Center Point) --
    // Wc_0 = Wm_0 + (1 - alpha^2 + beta)
    Wc_[0] = Wm_[0] + (1.0 - ALPHA * ALPHA + BETA);

    // -- Weights for remaining sigma points --
    // Wi = 1 / (2 * (n + lambda))
    double weight = 0.5 / (nx_ + lambda_);
    
    for (int i = 1; i < num_sigmas; ++i) {
        Wm_[i] = weight;
        Wc_[i] = weight;
    }
    std::cout << "UKF Constructor: TODO - Implement filter initialization" << lambda_ << std::endl;
}

// ============================================================================
// SIGMA POINT GENERATION
// ============================================================================

/**
 * @brief Generate sigma points from mean and covariance
 * 
 * STUDENT TODO:
 * 1. Start with the mean as the first sigma point
 * 2. Compute Cholesky decomposition of covariance
 * 3. Generate 2*n symmetric sigma points around the mean
 */
std::vector<Eigen::VectorXd> UKF::generateSigmaPoints(const Eigen::VectorXd& mean,
                                                       const Eigen::MatrixXd& cosv) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    std::vector<Eigen::VectorXd> sigma_points;
    int num_sigmas = 2 * nx_ + 1;
    sigma_points.resize(num_sigmas);

    // First sigma point is the mean itself
    sigma_points[0] = mean;

    // Calculate square root of matrix P_scaled = (n + lambda) * P
    // Using Cholesky decomposition 
    Eigen::MatrixXd P_scaled = (nx_ + lambda_) * cosv;
    Eigen::LLT<Eigen::MatrixXd> lltOfP(P_scaled);

    if (lltOfP.info() == Eigen::NumericalIssue) {
        // If matrix is not positive definite, handle error (fallback to mean)
        std::cerr << "UKF Error: Covariance matrix decomposition failed!" << std::endl;
        std::fill(sigma_points.begin(), sigma_points.end(), mean);
        return sigma_points;
    }

    Eigen::MatrixXd L = lltOfP.matrixL();

    // Generate remaining sigma points
    // X_i     = mean + row_of_L
    // X_{i+n} = mean - row_of_L
    for (int i = 0; i < nx_; ++i) {
        sigma_points[i + 1]       = mean + L.col(i);
        sigma_points[nx_ + i + 1] = mean - L.col(i);
    }

    return sigma_points;
}

// ============================================================================
// PROCESS MODEL
// ============================================================================

/**
 * @brief Apply motion model to a state vector
 * 
 * STUDENT TODO:
 * 1. Updates position: x' = x + dx, y' = y + dy
 * 2. Updates orientation: theta' = theta + dtheta (normalized)
 * 3. Updates velocities: vx' = dx/dt, vy' = dy/dt
 */
Eigen::VectorXd UKF::processModel(const Eigen::VectorXd& state, double dt,
                                  double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    Eigen::VectorXd new_state = state;
    
    // Update the position by directly adding the odometry displacement components
    new_state(0) += dx;
    new_state(1) += dy;
    
    // Update the orientation and ensure it stays within the normalized range of [-pi, pi]
    new_state(2) += dtheta;
    new_state(2) = normalizeAngle(new_state(2));
    
    // Compute the velocities based on the displacement over the time step
    // Only perform division if the time step is significant enough to avoid numerical instability
    if (dt > 1e-6) {
        new_state(3) = dx / dt;
        new_state(4) = dy / dt;
    } else {
        // Assume the robot is stationary if the time step is negligible
        new_state(3) = 0.0;
        new_state(4) = 0.0;
    }

    return new_state;

}

// ============================================================================
// MEASUREMENT MODEL
// ============================================================================

/**
 * @brief Predict measurement given current state and landmark
 * 
 * STUDENT TODO:
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
 * 
 * STUDENT TODO:
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
 * 
 * STUDENT TODO:
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
    
    // Iterate through all available landmark observations to update the state sequentially
    for (const auto& obs : landmark_observations) {
        // Extract observation data: ID and relative position (x, y)
        int id = std::get<0>(obs);
        double l_x_obs = std::get<1>(obs);
        double l_y_obs = std::get<2>(obs);
        
        // Skip unknown landmarks if they are not in our map
        if (!hasLandmark(id)) {
            continue;
        }

        // Generate sigma points based on the current predicted state
        // These points capture the distribution of our prediction before measurement update
        std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);

        // Transform sigma points into the measurement space
        // This predicts where we would see the landmark for each sigma point
        std::vector<Eigen::VectorXd> z_sigmas;
        z_sigmas.resize(2 * nx_ + 1);
        
        for (size_t i = 0; i < sigma_points.size(); ++i) {
            z_sigmas[i] = measurementModel(sigma_points[i], id);
        }

        // Calculate the mean of the predicted measurements
        Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(nz_);
        for (size_t i = 0; i < z_sigmas.size(); ++i) {
            z_pred += Wm_[i] * z_sigmas[i];
        }

        // Calculate the measurement covariance matrix (S) and the cross-covariance matrix (Pxz)
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nz_, nz_);
        Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(nx_, nz_);

        for (size_t i = 0; i < sigma_points.size(); ++i) {
            // Difference in state (State Residual)
            Eigen::VectorXd x_diff = sigma_points[i] - x_;
            x_diff(2) = normalizeAngle(x_diff(2)); // Normalize angle difference

            // Difference in measurement (Measurement Residual)
            Eigen::VectorXd z_diff = z_sigmas[i] - z_pred;
            
            // Accumulate weighted covariances
            S += Wc_[i] * (z_diff * z_diff.transpose());
            Pxz += Wc_[i] * (x_diff * z_diff.transpose());
        }

        // Add measurement noise to S matrix
        S += R_;

        // Compute the Kalman Gain
        // K = Pxz * S_inverse
        Eigen::MatrixXd K = Pxz * S.inverse();

        // Construct the actual measurement vector from sensor data
        Eigen::VectorXd z_actual(2);
        z_actual << l_x_obs, l_y_obs;
        
        // Calculate the innovation (residual): Real Measurement - Predicted Measurement
        Eigen::VectorXd innovation = z_actual - z_pred;
        
        // Update the state estimate using the Kalman Gain and innovation
        x_ += K * innovation;
        
        // Update the state covariance matrix
        P_ -= K * S * K.transpose();
        
        // Ensure the final orientation angle is normalized
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
