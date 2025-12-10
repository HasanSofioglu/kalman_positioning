#include "kalman_positioning/landmark_manager.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>

/**
 * STUDENT ASSIGNMENT: Landmark Manager Implementation
 * 
 * This class manages landmark positions loaded from a CSV file.
 * Students should implement the methods for loading, querying, and 
 * managing landmark data.
 */

// ============================================================================
// CSV FILE LOADING
// ============================================================================

/**
 * @brief Load landmarks from CSV file
 * 
 * STUDENT TODO:
 * 1. Open the CSV file at csv_path
 * 2. Parse each line as: id,x,y
 * 3. Handle comments (lines starting with #)
 * 4. Store landmark positions in the landmarks_ map
 * 5. Return true if successful, false otherwise
 */
bool LandmarkManager::loadFromCSV(const std::string& csv_path) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open landmark file: " << csv_path << std::endl;
        return false;
    }
    
    landmarks_.clear();
    std::string line;
    
    
    // Read line by line
    while (std::getline(file, line)) {
        // Skip empty lines or comments (starting with #)
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string segment;
        std::vector<std::string> parts;

        // Split line by comma ','
        while(std::getline(ss, segment, ',')) {
            parts.push_back(segment);
        }

        // We expect at least 3 parts: id, x, y
        if (parts.size() >= 3) {
            try {
                int id = std::stoi(parts[0]);
                double x = std::stod(parts[1]);
                double y = std::stod(parts[2]);
                
                // Store in map: id -> (x, y)
                landmarks_[id] = std::make_pair(x, y);
            } catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << " -> " << e.what() << std::endl;
                continue;
            }
        }
    }
    
    file.close();
    
    if (landmarks_.empty()) {
        std::cerr << "Warning: No landmarks loaded from " << csv_path << std::endl;
        return false;
    }
    
    std::cout << "Loaded " << landmarks_.size() << " landmarks from " << csv_path << std::endl;
    return true;
}

// ============================================================================
// LANDMARK QUERIES
// ============================================================================

/**
 * @brief Get all landmarks
 */
const std::map<int, std::pair<double, double>>& LandmarkManager::getLandmarks() const {
    return landmarks_;
}

/**
 * @brief Get landmark position by ID
 */
std::pair<double, double> LandmarkManager::getLandmark(int id) const {
    auto it = landmarks_.find(id);
    if (it != landmarks_.end()) {
        return it->second;
    }
    return {0.0, 0.0};
}

/**
 * @brief Check if landmark exists
 */
bool LandmarkManager::hasLandmark(int id) const {
    return landmarks_.find(id) != landmarks_.end();
}

// ============================================================================
// SPATIAL QUERIES
// ============================================================================

/**
 * @brief Get landmarks within a certain radius of a point
 * 
 * STUDENT TODO:
 * 1. Iterate through all landmarks
 * 2. Calculate distance from (x, y) to each landmark
 * 3. Add landmarks within radius to result vector
 * 4. Return the vector of landmark IDs
 */
std::vector<int> LandmarkManager::getLandmarksInRadius(double x, double y, double radius) const {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    std::vector<int> result;
    // Iterate through all loaded landmarks
    for (const auto& [id, pos] : landmarks_) {
        // Calculate Euclidean distance: sqrt((x1-x2)^2 + (y1-y2)^2)
        double dist = distance(x, y, pos.first, pos.second);
        
        // If within radius, add ID to results
        if (dist <= radius) {
            result.push_back(id);
        }
    }
    
    return result;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Calculate Euclidean distance between two points
 */
double LandmarkManager::distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}
