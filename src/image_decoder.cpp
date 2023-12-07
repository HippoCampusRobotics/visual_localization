#include <rclcpp/rclcpp.hpp>

class ImageDecoder : public rclcpp::Node {
 public:
  ImageDecoder(const rclcpp::NodeOptions &options)
      : Node("image_decoder", options) {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}
