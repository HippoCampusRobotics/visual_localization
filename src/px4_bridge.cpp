#include "px4_bridge.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static constexpr double kPi = 3.141592653589793238463;
static const Eigen::Quaterniond q_ned_enu{
    hippo_common::tf2_utils::EulerToQuaternion(kPi, 0.0, kPi / 2.0)};
static const Eigen::Quaterniond q_flu_frd{
    hippo_common::tf2_utils::EulerToQuaternion(kPi, 0.0, 0.0)};

namespace visual_localization {
Px4Bridge::Px4Bridge(rclcpp::NodeOptions const &_options)
    : Node("px4_bridge", _options) {
  // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS px4_qos{1};
  px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

  vehicle_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SystemDefaultsQoS());

  visual_odometry_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "fmu/in/vehicle_visual_odometry", px4_qos);

  vehicle_velocity_inertial_debug_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
          "~/debug/velocity_inertial", rclcpp::SystemDefaultsQoS());

  px4_vehicle_odometry_sub_ =
      create_subscription<px4_msgs::msg::VehicleOdometry>(
          "fmu/out/vehicle_odometry", px4_qos,
          std::bind(&Px4Bridge::OnOdometry, this, std::placeholders::_1));

  visual_pose_cov_stamped_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "vision_pose_cov", rclcpp::SystemDefaultsQoS(),
          std::bind(&Px4Bridge::OnPoseCov, this, std::placeholders::_1));

  vehicle_odom_update_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20),
                           std::bind(&Px4Bridge::OnUpdateOdometry, this));
}

void Px4Bridge::OnOdometry(
    px4_msgs::msg::VehicleOdometry::ConstSharedPtr _msg) {
  px4_odometry_updated_ = true;
  for (size_t i = 0; i < 3; ++i) {
    velocity_px4_(i) = _msg->velocity.at(i);
    position_px4_(i) = _msg->position.at(i);
    body_rates_px4_(i) = _msg->angular_velocity.at(i);
  }
  orientation_px4_.w() = _msg->q.at(0);
  orientation_px4_.x() = _msg->q.at(1);
  orientation_px4_.y() = _msg->q.at(2);
  orientation_px4_.z() = _msg->q.at(3);
}

// px4 msg to odometry msg in ROS
void Px4Bridge::OnUpdateOdometry() {
  if (!px4_odometry_updated_) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No odometry received from px4. Cannot publish odometry.");
    return;
  }
  px4_odometry_updated_ = false;

  rclcpp::Time stamp = now();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(position_px4_, pose.pose.position);
  hippo_common::convert::EigenToRos(orientation_px4_, pose.pose.orientation);
  pose.pose = hippo_common::tf2_utils::PosePx4ToRos(pose.pose);

  // velocity in inertial coordinate system in ROS
  Eigen::Vector3d velocity_ros = q_ned_enu.inverse() * velocity_px4_;
  geometry_msgs::msg::Vector3Stamped velocity_inertial_msg;
  velocity_inertial_msg.header.stamp = stamp;
  velocity_inertial_msg.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(velocity_ros, velocity_inertial_msg.vector);
  vehicle_velocity_inertial_debug_pub_->publish(velocity_inertial_msg);

  // transform velocity from inertial coordinate system in ROS to body frame
  velocity_ros =
      (q_ned_enu.inverse() * orientation_px4_ * q_flu_frd.inverse()).inverse() *
      velocity_ros;

  Eigen::Vector3d angular_velocity_ros = q_flu_frd * body_rates_px4_;

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  odometry.child_frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  odometry.pose.pose = pose.pose;
  hippo_common::convert::EigenToRos(velocity_ros, odometry.twist.twist.linear);
  hippo_common::convert::EigenToRos(angular_velocity_ros,
                                    odometry.twist.twist.angular);
  vehicle_odometry_pub_->publish(odometry);
}

// visual localization pose with covariance stamped in ROS to px4
void Px4Bridge::OnPoseCov(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr _msg) {
  px4_msgs::msg::VehicleOdometry visual_odometry_msg;
  visual_odometry_msg.pose_frame =
      px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  visual_odometry_msg.velocity_frame =
      px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
  visual_odometry_msg.timestamp =
      now().nanoseconds() * 1e-3;  // Why not time stamp from ekf?
  visual_odometry_msg.timestamp_sample = now().nanoseconds() * 1e-3;

  auto ros_pose = _msg->pose.pose;

  geometry_msgs::msg::Pose px4_pose =
      hippo_common::tf2_utils::PoseRosToPx4(ros_pose);
  visual_odometry_msg.position = {(float)px4_pose.position.x,
                                  (float)px4_pose.position.y,
                                  (float)px4_pose.position.z};
  visual_odometry_msg.q = {
      (float)px4_pose.orientation.w, (float)px4_pose.orientation.x,
      (float)px4_pose.orientation.y, (float)px4_pose.orientation.z};

  // auto ros_covariance = _msg->pose.covariance;
  // todo: convert covariance to px4 as well
  // ...
  // position_covariance = ros_covariance...;
  // orientation_covariance = ros_covariance...;
  // // velocity_covariance: do not have this here

  // position_covariance = q_ned_enu.toRotationMatrix() * position_covariance *
  //                       q_ned_enu.inverse().toRotationMatrix();
  // velocity_covariance = q_ned_enu.toRotationMatrix() * velocity_covariance *
  //                       q_ned_enu.inverse().toRotationMatrix();
  // // todo: properly transform orientation noise from ROS coordinate systems
  // to
  // // px4 coordinate systems
  // for (int i = 0; i < 3; i++) {  // rows
  //   visual_odometry.position_variance[i] = (float)position_covariance(i, i);
  //   visual_odometry.orientation_variance[i] =
  //       (float)orientation_covariance(i, i);
  //   visual_odometry.velocity_variance[i] = (float)velocity_covariance(i, i);
  // }

  visual_odometry_pub_->publish(visual_odometry_msg);
}

}  // namespace visual_localization

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<visual_localization::Px4Bridge>(options);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}