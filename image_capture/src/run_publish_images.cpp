/*
 * file: run_publish_images.cpp
 * purpose: Simple ROS2 node which publishes an image over the transport topic.
 */

#include <chrono>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:

  explicit ImagePublisher(std::string const & filepath)
    : Node("image_publisher"),
      mFilepath(filepath),
      mImage(cv_bridge::CvImage(std_msgs::msg::Header(),
                                "bgr8",
                                cv::imread(mFilepath,
                                           cv::IMREAD_COLOR)).toImageMsg())
  {
  }

  void start()
  {
    image_transport::ImageTransport it(shared_from_this());
    mPublisher = it.advertise("asar_image", 1);
    mTimer = create_wall_timer(1s, std::bind(&ImagePublisher::publishCallback, this));
  }

private:

  void publishCallback()
  {
    mPublisher.publish(mImage);
  }

  std::string mFilepath;
  sensor_msgs::msg::Image::SharedPtr mImage;
  image_transport::Publisher mPublisher;
  rclcpp::TimerBase::SharedPtr mTimer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string const FILENAME{"/home/ssnover/asar2.jpg"};

  auto image = cv::imread(FILENAME, cv::IMREAD_COLOR);
  cv::namedWindow("Test Image", cv::WINDOW_NORMAL);
  cv::imshow("Test Image", image);
  cv::waitKey(0);
  cv::destroyAllWindows();


  auto node = std::make_shared<ImagePublisher>(FILENAME);
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
}