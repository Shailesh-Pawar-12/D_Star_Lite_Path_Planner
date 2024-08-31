#ifndef DSTAR_NODE_HPP
#define DSTAR_NODE_HPP

#include "dstar_lite/dstar_lite.hpp"
#include <dstar_lite_interfaces/action/pathfinding.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <thread>

using namespace std::chrono_literals;
// ANSI escape code for green color
#define ANSI_COLOR_GREEN "\033[32m"
// ANSI escape code to reset color
#define ANSI_COLOR_RESET "\033[0m"

class DStarNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit DStarNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void on_shutdown();
  
protected:
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  using Pathfinding = dstar_lite_interfaces::action::Pathfinding;
  using GoalHandlePathfinding = rclcpp_action::ServerGoalHandle<Pathfinding>;

  rclcpp_action::Server<Pathfinding>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;  
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  std::shared_ptr<DStarLitePlanner> planner_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  int marker_id = 0;

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const Pathfinding::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePathfinding> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandlePathfinding> goal_handle);
  void publish_marker(int x, int y, const std::string &frame_id, float r, float g, float b);
  void publish_path(const std::vector<std::pair<int, int>> &path);
  void execute(const std::shared_ptr<GoalHandlePathfinding> goal_handle);

};

#endif // DSTAR_NODE_HPP
