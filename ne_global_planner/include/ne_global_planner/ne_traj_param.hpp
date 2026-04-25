#ifndef NE_TRAJ_PARAM_HPP
#define NE_TRAJ_PARAM_HPP
#include "Eigen/Dense"
#include <memory>
#include <iostream>
namespace traj_param
{
    
    struct PathState 
    {
      Eigen::Vector2d pos;
      double theta = 0;
      double delta_theta = 0;
      double deta_s = 0;  
    };
    struct TimedTrajectoryPoint
    {
        Eigen::Vector3d state;
        double time = 0;
    };
    struct Trajectory
    {
      std::vector<Eigen::Vector2d> raw_path;
      std::vector<Eigen::Vector2d > optimized_path;
      std::vector<PathState> path_states;
      std::vector<TimedTrajectoryPoint> timed_trajectory;
      
       Eigen::Vector3d start_state = Eigen::Vector3d::Zero();  // Start state (x_vel, y_vel, omega)
        Eigen::Vector3d final_state = Eigen::Vector3d::Zero();  // Final state
        Eigen::Vector3d start_state_XYTheta = Eigen::Vector3d::Zero(); // Start pose (x,y,theta)
        Eigen::Vector3d final_state_XYTheta = Eigen::Vector3d::Zero(); // Final pose
        
        // Path characteristics
        double total_time = 0;      // Total time
        double total_length = 0;    // Total length
        double weighted_length = 0; // Weighted length (considering angle changes)
        
        // Additional information
        bool if_cut = false;  // Whether trajectory is truncated
        std::vector<Eigen::Vector3d> UnOccupied_positions; // Unoccupied positions sequence
        double UnOccupied_initT = 0.0; // Initial time
    };
    
}
#endif