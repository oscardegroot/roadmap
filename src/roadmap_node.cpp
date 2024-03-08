
#include <roadmap/autoware/autoware_lanelet_converter.h>
// #include <roadmap/autoware/autoware_path_forward_interface.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  // auto autoware_interface = std::make_shared<AutowarePathForwardInterface>(roadmap);
  auto autoware_interface = std::make_shared<AutowareLaneletConverter>();
  autoware_interface->initialize();

  rclcpp::spin(autoware_interface);

  rclcpp::shutdown();

  return 0;
}
