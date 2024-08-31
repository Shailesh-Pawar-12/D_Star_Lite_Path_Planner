#include "dstar_lite/dstar_node.hpp"

DStarNode::DStarNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("dstar_node", options) {}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
DStarNode::on_configure(const rclcpp_lifecycle::State &) {
  using Pathfinding = dstar_lite_interfaces::action::Pathfinding;
  action_server_ = rclcpp_action::create_server<Pathfinding>(
      shared_from_this(), "pathfinding",
      std::bind(&DStarNode::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&DStarNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&DStarNode::handle_accepted, this, std::placeholders::_1));

  map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&DStarNode::map_callback, this, std::placeholders::_1));

  marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "start_goal_position", 10);

  path_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("d_star_planned_path", 10);
  RCLCPP_INFO(this->get_logger(), "Configuring");
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
DStarNode::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Activating");
  marker_publisher_->on_activate();
  path_publisher_->on_activate();
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
DStarNode::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Deactivating");
  marker_publisher_->on_deactivate();
  path_publisher_->on_deactivate();
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
DStarNode::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up");
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
DStarNode::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Shutting down");
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void DStarNode::on_shutdown() {
  on_deactivate(get_current_state());
  on_cleanup(get_current_state());
  on_shutdown(get_current_state());
}

void DStarNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = msg;

  int width = msg->info.width;
  int height = msg->info.height;
  planner_ = std::make_shared<DStarLitePlanner>(width, height);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = x + y * width;
      if (msg->data[index] == 100) {
        planner_->set_obstacle(x, y);
      } else if (msg->data[index] == 0) {
        planner_->remove_obstacle(x, y);
      }
    }
  }
}

rclcpp_action::GoalResponse
DStarNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const Pathfinding::Goal> goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received goal request with start (%f, %f) and goal (%f, %f)",
              goal->start.pose.position.x, goal->start.pose.position.y,
              goal->goal.pose.position.x, goal->goal.pose.position.y);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DStarNode::handle_cancel(
    const std::shared_ptr<GoalHandlePathfinding> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DStarNode::handle_accepted(
    const std::shared_ptr<GoalHandlePathfinding> goal_handle) {
  std::thread{std::bind(&DStarNode::execute, this, std::placeholders::_1),
              goal_handle}
      .detach();
}

void DStarNode::publish_marker(int x, int y, const std::string &frame_id,
                               float r, float g, float b) {
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "dstar_node";
  marker.id = marker_id++;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.1;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;
  marker_publisher_->publish(marker);
}

void DStarNode::publish_path(const std::vector<std::pair<int, int>> &path) {
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = this->now();

  for (const auto &point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_publisher_->publish(path_msg);
}

void DStarNode::execute(
    const std::shared_ptr<GoalHandlePathfinding> goal_handle) {

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Pathfinding::Feedback>();
  auto result = std::make_shared<Pathfinding::Result>();

  if (!current_map_) {
    RCLCPP_WARN(get_logger(), "No map received. Unable to plan.");
    feedback->progress = 0.0;
    goal_handle->publish_feedback(feedback);
    result->success = false;
    result->message = "No map received. Unable to plan.";
    goal_handle->abort(result);
    return;
  }

  int start_x = static_cast<int>(goal->start.pose.position.x);
  int start_y = static_cast<int>(goal->start.pose.position.y);
  int goal_x = static_cast<int>(goal->goal.pose.position.x);
  int goal_y = static_cast<int>(goal->goal.pose.position.y);

  publish_marker(start_x, start_y, "Start", 0.0, 1.0,
                 0.0); // Green marker for start
  publish_marker(goal_x, goal_y, "Goal", 1.0, 0.0, 0.0); // Red marker for goal

  bool plan_response = planner_->plan(start_x, start_y, goal_x, goal_y);
  if (plan_response) {
    const auto path = planner_->get_path();
    std::string path_str = "Path: ";
    for (const auto &point : path) {
      path_str += "(" + std::to_string(point.first) + ", " +
                  std::to_string(point.second) + ") ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", path_str.c_str());

    publish_path(path);

    feedback->progress = 100.0;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Pathfinding completed successfully.";
    std::cout << ANSI_COLOR_GREEN
              << "------------- Path Successfully Generated -------------"
              << ANSI_COLOR_RESET << std::endl;
    goal_handle->succeed(result);
  } else {
    feedback->progress = 0.0;
    goal_handle->publish_feedback(feedback);
    result->success = false;
    result->message = "Provided Start or Goal not valid.";
    goal_handle->abort(result);
  }
}
