#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "gpmp2_planner/srv/plan_path.hpp"

#include <opencv2/opencv.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <gpmp2/kinematics/Pose2MobileBase.h>
#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
#include <gpmp2/obstacle/PlanarSDF.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h>
#include <gpmp2/gp/GaussianProcessPriorPose2.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2.h>

// Represents gtsam::Symbol('x',i)
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;

class GPMP2ServiceNode : public rclcpp::Node
{
public:
  GPMP2ServiceNode() : Node("gpmp2_service_node")
  {
    service_ = this->create_service<gpmp2_planner::srv::PlanPath>(
      "plan_path",
      std::bind(&GPMP2ServiceNode::handle_plan, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "GPMP2 Service Node is ready and waiting for planning requests");
  }

private:
  void handle_plan(
    const std::shared_ptr<gpmp2_planner::srv::PlanPath::Request> request,
    std::shared_ptr<gpmp2_planner::srv::PlanPath::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received planning request");
    
    const auto& map = request->map;
    const auto& start = request->start;
    const auto& goal = request->goal;
    
    RCLCPP_INFO(this->get_logger(), "Map size: %d x %d", map.info.width, map.info.height);
    RCLCPP_INFO(this->get_logger(), "Start: (%.2f, %.2f)", start.position.x, start.position.y);
    RCLCPP_INFO(this->get_logger(), "Goal: (%.2f, %.2f)", goal.position.x, goal.position.y);
    
    // === Convert map to SDF matrix ===
    gtsam::Matrix sdf_matrix = computeSDF(map);
    RCLCPP_INFO(this->get_logger(), "SDF computed successfully");
    
    // === Create Robot Model (TurtleBot3 Burger - Pose2 Mobile Base) ===
    double robot_radius = 0.12;  // TurtleBot3 Burger radius ~10.5cm + safety

    // Create kinematics model (Pose2MobileBase handles x, y, theta)
    gpmp2::Pose2MobileBase abstract_robot;

    // Create body spheres (one sphere covering the robot footprint)
    gpmp2::BodySphereVector body_spheres;
    body_spheres.push_back(gpmp2::BodySphere(
        0,                        // link_id (base link)
        robot_radius,             // radius
        gtsam::Point3(0, 0, 0)    // center at robot origin
    ));

    // Create physical robot model
    gpmp2::Pose2MobileBaseModel robot(abstract_robot, body_spheres);

    RCLCPP_INFO(this->get_logger(), "Robot model created (Pose2MobileBase, radius=%.3f m)", robot_radius);
    
    // Create PlanarSDF (2D signed distance field)
    gtsam::Point2 origin_2d(
        map.info.origin.position.x,
        map.info.origin.position.y
    );

    gpmp2::PlanarSDF sdf(
        origin_2d,
        map.info.resolution,
        sdf_matrix
    );

    RCLCPP_INFO(this->get_logger(), "PlanarSDF created (origin: %.2f, %.2f, resolution: %.3f)",
                origin_2d.x(), origin_2d.y(), map.info.resolution);
    
    // === Extract start and goal as Pose2 ===
    double start_yaw = quaternionToYaw(start.orientation);
    double goal_yaw = quaternionToYaw(goal.orientation);
    
    gtsam::Pose2 start_conf(start.position.x, start.position.y, start_yaw);
    gtsam::Pose2 goal_conf(goal.position.x, goal.position.y, goal_yaw);
    
    RCLCPP_INFO(this->get_logger(), "Start config: (%.2f, %.2f, %.2f rad)", 
                start_conf.x(), start_conf.y(), start_conf.theta());
    RCLCPP_INFO(this->get_logger(), "Goal config: (%.2f, %.2f, %.2f rad)", 
                goal_conf.x(), goal_conf.y(), goal_conf.theta());
    
    // Planning Parameters
    double Qc_scale = 0.01;
    gtsam::Matrix Qc_matrix = Qc_scale * gtsam::Matrix::Identity(3,3);
    auto Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc_matrix);

    double epsilon = 0.5;
    double cost_sigma = 0.1;

    double pose_sigma = 0.001;
    double vel_sigma = 0.001;

    // === Initialize Trajectory ===
    int num_waypoints = 12;
    double total_time = 10.0;
    double delta_t = total_time / (num_waypoints - 1);
    
    gtsam::Values initial_values = initializeStraightLineTrajectory(
        start_conf, 
        goal_conf, 
        num_waypoints, total_time
    );
    
    RCLCPP_INFO(this->get_logger(), 
                "Initialized trajectory with %d waypoints (delta_t=%.2f sec)",
                num_waypoints, delta_t);
    
    gtsam::Pose2 first = initial_values.at<gtsam::Pose2>(gtsam::Symbol('x', 0));
    gtsam::Pose2 last = initial_values.at<gtsam::Pose2>(gtsam::Symbol('x', num_waypoints-1));
    
    RCLCPP_INFO(this->get_logger(), "First waypoint: (%.2f, %.2f, %.2f)", 
                first.x(), first.y(), first.theta());
    RCLCPP_INFO(this->get_logger(), "Last waypoint: (%.2f, %.2f, %.2f)", 
                last.x(), last.y(), last.theta());
    
    // === Build factor graph ===
    gtsam::NonlinearFactorGraph graph;

    auto pose_prior_noise = gtsam::noiseModel::Isotropic::Sigma(3, pose_sigma);
    auto vel_prior_noise = gtsam::noiseModel::Isotropic::Sigma(3, vel_sigma);

    // Start constraints
    gtsam::Vector start_vel = gtsam::Vector::Zero(3);
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(X(0), start_conf, pose_prior_noise));
    graph.add(gtsam::PriorFactor<gtsam::Vector>(V(0), start_vel, vel_prior_noise));

    // Goal constraints
    gtsam::Vector goal_vel = gtsam::Vector::Zero(3);
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(X(num_waypoints - 1), goal_conf, pose_prior_noise));
    graph.add(gtsam::PriorFactor<gtsam::Vector>(V(num_waypoints - 1), goal_vel, vel_prior_noise));

    // GP Priors (Smoothness)
    for (int i = 0; i < num_waypoints - 1; i++) {
      graph.add(gpmp2::GaussianProcessPriorPose2(X(i), V(i), X(i+1), V(i+1), delta_t, Qc_model));
    }

    // Obstacle factors
    for (int i = 0; i < num_waypoints; i++){
      graph.add(gpmp2::ObstaclePlanarSDFFactorPose2MobileBase(X(i), robot, sdf, cost_sigma, epsilon));
    }

    // Interpolated obstacle factors (between waypoints)
    int num_interpolations = 3;
    for (int i = 0; i < num_waypoints - 1; i++) {
      for (int j = 1; j <= num_interpolations; j++) {
        double tau = static_cast<double>(j) / (num_interpolations + 1);
        graph.add(gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase(X(i), V(i), X(i+1), V(i+1), robot, sdf, cost_sigma, epsilon, Qc_model, delta_t, tau));
      }
    }

    // === Optimize ===
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(100);
    params.setRelativeErrorTol(1e-5);
    params.setAbsoluteErrorTol(1e-5);

    gtsam::Values optimized_values;
    bool optimization_success = true;

    try {
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);
      optimized_values = optimizer.optimize();
    } 
    catch (const std::exception& e) {
      optimization_success = false;
      RCLCPP_ERROR(this->get_logger(), "Optimization failed: %s", e.what());
      optimized_values = initial_values;
    }

    // === Extract Path ===
    response->path = extractProperGPPath(
      optimized_values,
      num_waypoints,
      delta_t,
      Qc_model,
      0.05);
  
    response->success = optimization_success;
    response->message = optimization_success ? "GPMP2 optimization successful" : "Optimization failed";
  }

  rclcpp::Service<gpmp2_planner::srv::PlanPath>::SharedPtr service_;
  
  // Function to convert occupancy grid to an OpenCV binary image
  cv::Mat occupancyToBinaryCV(const nav_msgs::msg::OccupancyGrid& grid) {
    cv::Mat binary(grid.info.height, grid.info.width, CV_8UC1);
    
    for (size_t i = 0; i < grid.data.size(); i++) {
      int value = grid.data[i];
      binary.data[i] = (value >= 50) ? 255 : 0;
    }
    
    return binary;
  }
  
  // Function to calculate the SDF given an occupancy grid
  gtsam::Matrix computeSDF(const nav_msgs::msg::OccupancyGrid& grid) {
    cv::Mat binary = occupancyToBinaryCV(grid);
    
    cv::Mat dist_from_occupied;
    cv::distanceTransform(binary, dist_from_occupied, cv::DIST_L2, 5);
    
    cv::Mat dist_from_free;
    cv::distanceTransform(~binary, dist_from_free, cv::DIST_L2, 5);
    
    cv::Mat sdf_cv = dist_from_free - dist_from_occupied;
    
    gtsam::Matrix sdf(grid.info.height, grid.info.width);
    for (int row = 0; row < sdf_cv.rows; row++) {
      for (int col = 0; col < sdf_cv.cols; col++) {
        sdf(row, col) = sdf_cv.at<float>(row, col) * grid.info.resolution;
      }
    }
    
    return sdf;
  }
  
  // Function to convert quaternion to yaw angle
  double quaternionToYaw(const geometry_msgs::msg::Quaternion& quat) {
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // Function to convert yaw angles to quaternions for path
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion quat;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = std::sin(yaw / 2.0);
    quat.w = std::cos(yaw / 2.0);
    return quat;
  }

  // Function to create an initial straight line trajectory given a start, end, and number of waypoints
  gtsam::Values initializeStraightLineTrajectory(
    const gtsam::Pose2& start_pose,
    const gtsam::Pose2& goal_pose,
    int num_waypoints, double total_time)
  {
  gtsam::Values initial_values;
  double delta_t = total_time / (num_waypoints - 1);
  
  // FIRST PASS: Create all poses and store them
  std::vector<gtsam::Pose2> poses;
  poses.reserve(num_waypoints);
  
  for (int i = 0; i < num_waypoints; i++) {
    double t = static_cast<double>(i) / (num_waypoints - 1);
    
    double x = (1.0 - t) * start_pose.x() + t * goal_pose.x();
    double y = (1.0 - t) * start_pose.y() + t * goal_pose.y();
    
    double start_theta = start_pose.theta();
    double goal_theta = goal_pose.theta();
    
    double theta_diff = goal_theta - start_theta;
    if (theta_diff > M_PI) theta_diff -= 2.0 * M_PI;
    if (theta_diff < -M_PI) theta_diff += 2.0 * M_PI;
    
    double theta = start_theta + t * theta_diff;
    
    poses.push_back(gtsam::Pose2(x, y, theta));
  }
  
  // SECOND PASS: Insert poses and compute velocities using finite differences
  for (int i = 0; i < num_waypoints; i++) {
    // Insert pose
    initial_values.insert(X(i), poses[i]);
    
    // Compute velocity from finite differences
    gtsam::Vector vel(3);
    
    if (i == 0) {
      // Forward difference at start
      double dx = poses[1].x() - poses[0].x();
      double dy = poses[1].y() - poses[0].y();
      double dtheta = poses[1].theta() - poses[0].theta();
      
      // Normalize angle difference
      if (dtheta > M_PI) dtheta -= 2.0 * M_PI;
      if (dtheta < -M_PI) dtheta += 2.0 * M_PI;
      
      vel << dx / delta_t, dy / delta_t, dtheta / delta_t;
      
    } else if (i == num_waypoints - 1) {
      // Zero velocity at goal (stopped)
      vel << 0.0, 0.0, 0.0;
      
    } else {
      // Central difference in middle (most accurate)
      double dx = poses[i+1].x() - poses[i-1].x();
      double dy = poses[i+1].y() - poses[i-1].y();
      double dtheta = poses[i+1].theta() - poses[i-1].theta();
      
      // Normalize angle difference
      if (dtheta > M_PI) dtheta -= 2.0 * M_PI;
      if (dtheta < -M_PI) dtheta += 2.0 * M_PI;
      
      vel << dx / (2.0 * delta_t), 
             dy / (2.0 * delta_t), 
             dtheta / (2.0 * delta_t);
    }
    
    initial_values.insert(V(i), vel);
  }
  
  return initial_values;
  }

  // Function to extract the optimized gaussian process path in full from the interpolater
  nav_msgs::msg::Path extractProperGPPath(
      const gtsam::Values& optimized_values,
      int num_waypoints,
      double delta_t,
      const gtsam::noiseModel::Gaussian::shared_ptr& Qc_model,
      double spacing = 0.05)  // Extract point every 5cm
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    
    for (int i = 0; i < num_waypoints - 1; i++) {
      gtsam::Pose2 Xi = optimized_values.at<gtsam::Pose2>(X(i));
      gtsam::Vector Vi = optimized_values.at<gtsam::Vector>(V(i));
      gtsam::Pose2 Xi1 = optimized_values.at<gtsam::Pose2>(X(i+1));
      gtsam::Vector Vi1 = optimized_values.at<gtsam::Vector>(V(i+1));
      
      // Compute segment length for adaptive sampling
      double segment_length = std::hypot(Xi1.x() - Xi.x(), Xi1.y() - Xi.y());
      int num_samples = std::max(2, (int)(segment_length / spacing));
      
      // Sample along the segment using GPMP2's interpolator
      for (int j = 0; j < num_samples; j++) {
        double tau = static_cast<double>(j) / num_samples;
        
        // Create GPMP2 interpolator for this specific tau
        gpmp2::GaussianProcessInterpolatorPose2 interpolator(Qc_model, delta_t, tau);
        
        // Interpolate pose using GPMP2's exact GP formula
        gtsam::Pose2 interpolated = interpolator.interpolatePose(Xi, Vi, Xi1, Vi1);
        
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position.x = interpolated.x();
        pose_stamped.pose.position.y = interpolated.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = yawToQuaternion(interpolated.theta());
        
        path.poses.push_back(pose_stamped);
      }
    }
    
    // Add final waypoint
    gtsam::Pose2 final_pose = optimized_values.at<gtsam::Pose2>(X(num_waypoints-1));
    geometry_msgs::msg::PoseStamped final_stamped;
    final_stamped.header = path.header;
    final_stamped.pose.position.x = final_pose.x();
    final_stamped.pose.position.y = final_pose.y();
    final_stamped.pose.position.z = 0.0;
    final_stamped.pose.orientation = yawToQuaternion(final_pose.theta());
    path.poses.push_back(final_stamped);
    
    RCLCPP_INFO(this->get_logger(), 
      "Extracted %zu poses using GPMP2 GP interpolation (spacing ~%.2fm)", 
      path.poses.size(), spacing);
    
    return path;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GPMP2ServiceNode>();
  
  RCLCPP_INFO(node->get_logger(), "Spinning service node...");
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}