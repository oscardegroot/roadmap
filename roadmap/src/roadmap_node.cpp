
#include <roadmap.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  auto roadmap = std::make_shared<Roadmap>();
  roadmap->Initialize();

  rclcpp::spin(roadmap);

  rclcpp::shutdown();

  return 0;
}
