#ifndef IMAGE_TRANSPORT_NODE_HPP
#define IMAGE_TRANSPORT_NODE_HPP

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/image.hpp>

class ImageTransportNode : public rclcpp::Node
{
  public:
    ImageTransportNode();
  	bool initialize();

   private:
   	std::vector<image_transport::Subscriber> it_subscribers_;
  	std::vector<image_transport::Publisher> it_publishers_;
};

#endif // IMAGE_TRANSPORT_NODE_HPP
