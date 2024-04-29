#include <carla_infrastructure/image_transport_node.hpp>

ImageTransportNode::ImageTransportNode()
  : Node("image_transport_node")
{
  this->declare_parameter<std::string>("sensor_params", "");
  this->declare_parameter<std::string>("in_transport", "compressed");
}

bool ImageTransportNode::initialize()
{
  try
  {
    std::string params = this->get_parameter("sensor_params").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Reading sensor params in " << params);
    YAML::Node config = YAML::LoadFile(params);
    if(config["sensors"])
    {
      typedef void (image_transport::Publisher::* PublishMemFn)(
        const sensor_msgs::msg::Image::ConstSharedPtr &) const;
      PublishMemFn pub_mem_fn = &image_transport::Publisher::publish;

      config = config["sensors"]["rgb_camera"];
      for(unsigned int ii = 0; ii < config.size(); ++ii)
      {
        std::string out_topic = rclcpp::expand_topic_or_service_name(
          config[ii]["topic"].as<std::string>() + "/image_it",
          this->get_name(), this->get_namespace());

        auto pub = image_transport::create_publisher(this, out_topic);
        it_publishers_.push_back(pub);
      }

      for(const auto &pub : it_publishers_)
      {
        std::string in_topic = pub.getTopic();
        std::string remove_str = "_it";
        std::string::size_type i_str = in_topic.find(remove_str);
        if(i_str != std::string::npos)
        {
          in_topic.erase(i_str, remove_str.length());
        }

        auto sub = image_transport::create_subscription(
          this, in_topic, std::bind(pub_mem_fn, pub, std::placeholders::_1),
          this->get_parameter("in_transport").as_string());
        it_subscribers_.push_back(sub);
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Sensor info not found in config file, exiting...!");
      return false;
    }
  }
  catch(const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error reading config file, " << e.what());
    return false;
  }
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto it_node = std::make_shared<ImageTransportNode>();
  if(it_node->initialize())
  {
    rclcpp::spin(it_node);
  }
  rclcpp::shutdown();
  return 0;
}