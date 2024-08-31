// main.cpp
#include "dstar_lite/dstar_node.hpp"

void on_shutdown(std::shared_ptr<DStarNode> & d_star_lite_planner_node)
{
  d_star_lite_planner_node->on_shutdown();
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node_options = rclcpp::NodeOptions();  
  auto node =
      std::make_shared<DStarNode>(node_options);
    
  rclcpp::on_shutdown(std::bind(&on_shutdown, std::ref(node)));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
