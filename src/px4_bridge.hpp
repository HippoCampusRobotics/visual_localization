#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace visual_localization {
class Px4Bridge : public rclcpp::Node {
 public:
  explicit Px4Bridge(rclcpp::NodeOptions const &_options);

 private:
  void OnOdometry(px4_msgs::msg::VehicleOdometry::ConstSharedPtr _msg);
  void OnUpdateOdometry();
  void OnPoseCov(
      geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr _msg);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
      visual_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      vehicle_velocity_inertial_debug_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      visual_pose_cov_stamped_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      px4_vehicle_odometry_sub_;
  rclcpp::TimerBase::SharedPtr vehicle_odom_update_timer_;

  Eigen::Vector3d position_px4_;
  Eigen::Quaterniond orientation_px4_;
  Eigen::Vector3d velocity_px4_;
  Eigen::Vector3d body_rates_px4_;
  bool px4_odometry_updated_{false};

  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
}  // namespace visual_localization
