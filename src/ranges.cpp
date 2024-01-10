#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hippo_msgs/msg/range_measurement.hpp>
#include <hippo_msgs/msg/range_measurement_array.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unordered_map>

hippo_msgs::msg::RangeMeasurement createRangeMeasurement(int range_id,
                                                         double range) {
  auto msg = hippo_msgs::msg::RangeMeasurement();
  msg.range = range;
  msg.id = range_id;
  return msg;
}

class Ranges : public rclcpp::Node {
 public:
  Ranges(const rclcpp::NodeOptions &options) : Node("ranges", options) {
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    tag_ids_ = {{0, 0}, {1, 1}, {2, 2}, {3, 3}};
    ranges_pub_ =
        create_publisher<hippo_msgs::msg::RangeMeasurementArray>("ranges", qos);
    tag_transform_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "front_camera/tag_transforms", qos,
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          OnTagTransforms(msg);
        });
  }

 private:
  void OnTagTransforms(const tf2_msgs::msg::TFMessage::SharedPtr _msg) {
    hippo_msgs::msg::RangeMeasurementArray range_array_msg;
    for (const auto &tf_stamped : _msg->transforms) {
      auto source = tf_stamped.child_frame_id;
      source.erase(std::remove_if(source.begin(), source.end(),
                                  [](char c) { return !std::isdigit(c); }),
                   source.end());
      int tag_id = std::stoi(source);
      int range_id;
      try {
        range_id = tag_ids_.at(tag_id);

      } catch (const std::out_of_range &e) {
        RCLCPP_WARN(get_logger(),
                    "Tag with id %d not in tag_ids dict. Ignoring it.", tag_id);
        continue;
      }
      auto p = tf_stamped.transform.translation;
      double distance = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      auto measurement = createRangeMeasurement(range_id, distance);
      measurement.header = tf_stamped.header;
      range_array_msg.measurements.push_back(measurement);
    }
    if (!ranges_pub_) {
      RCLCPP_ERROR(get_logger(), "Publisher not initialized.");
      return;
    }
    range_array_msg.header.stamp = this->get_clock()->now();
    range_array_msg.header.frame_id = "range_sensor";
    ranges_pub_->publish(range_array_msg);
  }
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tag_transform_sub_;
  rclcpp::Publisher<hippo_msgs::msg::RangeMeasurementArray>::SharedPtr
      ranges_pub_;
  std::unordered_map<int, int> tag_ids_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<Ranges>(options);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
