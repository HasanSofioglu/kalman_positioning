#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "kalman_positioning/ukf.hpp"
#include "kalman_positioning/landmark_manager.hpp"
#include <memory>
#include <cmath>

/**
 * @brief Positioning node for UKF-based robot localization (Student Assignment)
 * 
 * This node subscribes to:
 *   - /robot_noisy: Noisy odometry (dead-reckoning)
 *   - /landmarks_observed: Noisy landmark observations
 * 
 * And publishes to:
 *   - /robot_estimated_odometry: Estimated pose and velocity from filter
 * 
 * STUDENT ASSIGNMENT:
 * Implement the Kalman filter logic to fuse odometry and landmark observations
 * to estimate the robot's true position.
 */
class PositioningNode : public rclcpp::Node {
public:
    PositioningNode() : Node("kalman_positioning_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Kalman Positioning Node");
        
        // Create subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_noisy",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::odometryCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_noisy");
        
        landmarks_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_observed",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::landmarksObservedCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_observed");
        
        // Create publisher
        estimated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/robot_estimated_odometry", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing to /robot_estimated_odometry");
        
        RCLCPP_INFO(this->get_logger(), "Kalman Positioning Node initialized successfully");
        // Declare the ROS parameters necessary for the filter configuration
        // giving them default values in case they are not provided by the launch file
        this->declare_parameter("landmarks_csv_path", "landmarks.csv");
        this->declare_parameter("process_noise_xy", 0.1);
        this->declare_parameter("process_noise_theta", 0.05);
        this->declare_parameter("measurement_noise_xy", 0.2);
        
        // Retrieve the parameter values to set up the system
        std::string csv_path = this->get_parameter("landmarks_csv_path").as_string();
        double p_noise_xy = this->get_parameter("process_noise_xy").as_double();
        double p_noise_theta = this->get_parameter("process_noise_theta").as_double();
        measurement_noise_xy_ = this->get_parameter("measurement_noise_xy").as_double();

        // Instantiate the Landmark Manager to handle map operations
        landmark_manager_ = std::make_shared<LandmarkManager>();
        
        // Attempt to load the landmark positions from the CSV file
        // We log an error if the file cannot be found, but continue execution
        if (!landmark_manager_->loadFromCSV(csv_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load landmarks from: %s", csv_path.c_str());
        } else {
             RCLCPP_INFO(this->get_logger(), "Landmarks successfully loaded from: %s", csv_path.c_str());
        }

        // Initialize the Unscented Kalman Filter with the noise parameters
        // We also pass the number of landmarks to correctly size the matrices
        int num_landmarks = landmark_manager_->getNumLandmarks();
        ukf_ = std::make_unique<UKF>(p_noise_xy, p_noise_theta, measurement_noise_xy_, num_landmarks);
        
        // Provide the filter with the known map data
        ukf_->setLandmarks(landmark_manager_->getLandmarks());
    }

private:
    // ============================================================================
    // SUBSCRIBERS AND PUBLISHERS
    // ============================================================================
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_obs_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub_;
    
    // ============================================================================
    // PLACEHOLDER: KALMAN FILTER STATE
    // ============================================================================
    // Students should implement a proper Kalman filter (e.g., UKF, EKF) 
    // with the following state:
    //   - Position: x, y (m)
    //   - Orientation: theta (rad)
    //   - Velocity: vx, vy (m/s)
    // And maintain:
    //   - State covariance matrix
    //   - Process noise covariance
    //   - Measurement noise covariance
    // Core filter components
    std::unique_ptr<UKF> ukf_;
    std::shared_ptr<LandmarkManager> landmark_manager_;
    
    // Helper variable for time calculation
    rclcpp::Time last_odom_time_;
    double measurement_noise_xy_;
    // ============================================================================
    // CALLBACK FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Callback for noisy odometry measurements
     * 
     * STUDENT TODO:
     * 1. Extract position (x, y) and orientation (theta) from the message
     * 2. Update the Kalman filter's prediction step with this odometry
     * 3. Publish the estimated odometry
     */
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), 
            "Odometry received: x=%.3f, y=%.3f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        
        // STUDENT ASSIGNMENT STARTS HERE
        // ========================================================================
        // Ensure the UKF instance exists before proceeding
        if (!ukf_) return;

        rclcpp::Time current_time = msg->header.stamp;

        // Initialize the last timestamp on the very first message received
        if (last_odom_time_.nanoseconds() == 0) {
            last_odom_time_ = current_time;
            return;
        }

        // Calculate the time elapsed since the last odometry update
        double dt = (current_time - last_odom_time_).seconds();
        
        // Perform a sanity check on the time step to avoid errors after simulation resets
        if (dt < 0.0 || dt > 1.0) {
            last_odom_time_ = current_time;
            return;
        }

        // Extract the linear and angular velocities from the message
        double v = msg->twist.twist.linear.x;
        double omega = msg->twist.twist.angular.z;

        // Calculate the displacement components based on the filter's current state
        // We use the estimated theta rather than the noisy odometry theta
        double current_theta = ukf_->getState()(2);
        
        double dx = v * dt * std::cos(current_theta);
        double dy = v * dt * std::sin(current_theta);
        double dtheta = omega * dt;

        // execute the UKF prediction step to propagate the state forward
        ukf_->predict(dt, dx, dy, dtheta);
        
        // Update the timestamp for the next iteration
        last_odom_time_ = current_time;

        // Publish the newly estimated state
        publishEstimatedOdometry(current_time, *msg);
       
    }
    
    /**
     * @brief Callback for landmark observations
     * 
     * STUDENT TODO:
     * 1. Parse the PointCloud2 data to extract landmark observations
     * 2. Update the Kalman filter's measurement update step with these observations
     * 3. Optionally publish the updated estimated odometry
     */
    void landmarksObservedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // STUDENT ASSIGNMENT STARTS HERE
        // ========================================================================
        // Check if the system is fully initialized
        if (!ukf_ || !landmark_manager_) return;

        std::vector<std::tuple<int, double, double, double>> observations;
        
        // Create iterators to traverse the PointCloud2 data fields
        // We look for x and y coordinates, and the landmark ID
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        
        // Based on the provided code structure, we assume the ID is stored in the 'id' field
        sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_id(*msg, "id");
        
        // Loop through all points in the cloud
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_id) {
            int id = static_cast<int>(*iter_id);
            double lx = static_cast<double>(*iter_x);
            double ly = static_cast<double>(*iter_y);
            
            // Store the observation with the current measurement noise parameter
            observations.emplace_back(id, lx, ly, measurement_noise_xy_);
        }

        // If valid observations were found, trigger the UKF update step
        if (!observations.empty()) {
            ukf_->update(observations);
        }
        
       
    }
    
    // ============================================================================
    // HELPER FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Convert quaternion to yaw angle
     * @param q Quaternion from orientation
     * @return Yaw angle in radians [-pi, pi]
     */
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle Input angle in radians
     * @return Normalized angle in [-pi, pi]
     */
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    /**
     * @brief Publish estimated odometry message
     * @param timestamp Message timestamp
     * @param odom_msg Odometry message to publish
     */
    void publishEstimatedOdometry(const rclcpp::Time& timestamp, 
                                  const nav_msgs::msg::Odometry& odom_msg) {
        nav_msgs::msg::Odometry estimated_odom = odom_msg;
        estimated_odom.header.stamp = timestamp;
        estimated_odom.header.frame_id = "map";
        estimated_odom.child_frame_id = "robot_estimated";
        
        // STUDENT TODO: Replace this with actual filter output
        // Set position, orientation, velocity, and covariance from your Kalman filter
        // Currently using placeholder values (noisy odometry)
        
        
        // STUDENT ASSIGNMENT: Fill with actual filter data
        // ========================================================================
        
        if (ukf_) {
            // Retrieve the estimated state [x, y, theta, vx, vy] and covariance from UKF
            Eigen::VectorXd x = ukf_->getState();
            Eigen::MatrixXd P = ukf_->getCovariance();
            
            // 1. Set Position
            estimated_odom.pose.pose.position.x = x(0);
            estimated_odom.pose.pose.position.y = x(1);
            estimated_odom.pose.pose.position.z = 0.0;
            
            // 2. Set Orientation (Convert Yaw angle to Quaternion)
            tf2::Quaternion q;
            q.setRPY(0, 0, x(2)); // Roll=0, Pitch=0, Yaw=theta
            estimated_odom.pose.pose.orientation = tf2::toMsg(q);
            
            // 3. Set Velocity
            estimated_odom.twist.twist.linear.x = x(3);
            estimated_odom.twist.twist.linear.y = x(4);
            
            // 4. Set Covariance (Map 5x5 UKF matrix to 6x6 ROS array)
            // Initialize array to zero first
            for(int i=0; i<36; i++) estimated_odom.pose.covariance[i] = 0.0;
            
            // Map the diagonal elements (Variance)
            estimated_odom.pose.covariance[0]  = P(0, 0); // x variance
            estimated_odom.pose.covariance[7]  = P(1, 1); // y variance
            estimated_odom.pose.covariance[35] = P(2, 2); // theta variance
            
            // Map relevant cross-covariance terms (x-y correlation)
            estimated_odom.pose.covariance[1]  = P(0, 1); // x-y covariance
            estimated_odom.pose.covariance[6]  = P(1, 0); // y-x covariance (symmetric)
        }
        
        // Publish the constructed messagesss
        estimated_odom_pub_->publish(estimated_odom);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositioningNode>());
    rclcpp::shutdown();
    return 0;
}
