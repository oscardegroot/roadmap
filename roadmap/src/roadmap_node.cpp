
#include <roadmap/roadmap.h>
// #include <roadmap/autoware/autoware_lanelet_converter.h>
// #include <roadmap/autoware/autoware_path_forward_interface.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  auto roadmap = std::make_shared<Roadmap>();
  roadmap->Initialize();

  // auto autoware_interface = std::make_shared<AutowarePathForwardInterface>(roadmap);
  // auto autoware_interface = std::make_shared<AutowareLaneletConverter>(roadmap);

  rclcpp::spin(roadmap);

  rclcpp::shutdown();

  return 0;
}
