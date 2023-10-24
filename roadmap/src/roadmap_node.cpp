
#include <roadmap.h>
#include <autoware_interface.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  auto roadmap = std::make_shared<Roadmap>();
  roadmap->Initialize();

  auto autoware_interface = std::make_shared<AutowareInterface>(roadmap);

  rclcpp::spin(roadmap);

  rclcpp::shutdown();

  return 0;
}
