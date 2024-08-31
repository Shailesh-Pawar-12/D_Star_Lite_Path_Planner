#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <dstar_lite_interfaces/action/pathfinding.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace dstar_lite_cpp
{
class DStarLiteActionClient : public rclcpp::Node
{
public:
  using DStarLite = dstar_lite_interfaces::action::Pathfinding;
  using GoalHandleDStarLite = rclcpp_action::ClientGoalHandle<DStarLite>;

  explicit DStarLiteActionClient(const rclcpp::NodeOptions & options)
  : Node("pathfinding_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<DStarLite>(
      this,
      "pathfinding");

    // Timer to periodically check if the action server is available
    auto timer_callback_lambda = [this](){ return this->send_goal(); };
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = DStarLite::Goal();
    goal_msg.start.pose.position.x = 1.0; 
    goal_msg.start.pose.position.y = 1.0;
    goal_msg.goal.pose.position.x = 5.0; 
    goal_msg.goal.pose.position.y = 7.0;
    std::cout << "goal_msg.start.pose.position.x = " << goal_msg.start.pose.position.x << std::endl; 

        RCLCPP_INFO(this->get_logger(), "Sending goal request with start (%f, %f) and goal (%f, %f)",
                    goal_msg.start.pose.position.x, goal_msg.start.pose.position.y,
                    goal_msg.goal.pose.position.x, goal_msg.goal.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<DStarLite>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleDStarLite::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleDStarLite::SharedPtr,
      const std::shared_ptr<const DStarLite::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(), "Received feedback: Progress = %f", feedback->progress);
    };

    send_goal_options.result_callback = [this](const GoalHandleDStarLite::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }

      // Process the result
      std::stringstream ss;
      ss << "Result received: ";
      ss << "Success: " << (result.result->success ? "true" : "false") << ", ";
      ss << "Message: " << result.result->message;
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());


      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<DStarLite>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};  // class DStarLiteActionClient

}  // namespace dstar_lite_cpp

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dstar_lite_cpp::DStarLiteActionClient>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}