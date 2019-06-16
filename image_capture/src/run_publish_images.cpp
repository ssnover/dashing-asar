/*
 * file: run_publish_images.cpp
 * purpose: Simple ROS2 node which publishes an image over the transport topic.
 */

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:

  ImagePublisher(std::string const & filepath)
    : Node("image_publisher"),
      mFilepath(filepath),
      mPublisher(create_publisher<std_msgs::msg::String>("/image_filepath", 5)),
      mTimer(create_wall_timer(1s, std::bind(&ImagePublisher::publishStringCallback, this)))
  {
  }

private:

  void publishStringCallback()
  {
    auto message = std_msgs::msg::String();
    message.data = mFilepath;
    mPublisher->publish(message);
  }

  std::string mFilepath;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPublisher;
  rclcpp::TimerBase::SharedPtr mTimer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>("hello world"));
  rclcpp::shutdown();
}