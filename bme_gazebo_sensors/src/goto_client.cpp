#include <memory>
#include <functional>
#include <thread>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/ass1goto.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class GoToClient : public rclcpp::Node
{
public:
  using Goto = action_interfaces::action::Ass1goto;
  using GoalHandleGoto = rclcpp_action::ClientGoalHandle<Goto>;

  explicit GoToClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("go_to_client", options), cancel_sent_(false)
  {
    using namespace  std::placeholders;

    this->action_client_ = rclcpp_action::create_client<Goto>(
      this,
      "goto");

    input_thread = std::thread(&GoToClient::user_input_loop, this);
    input_thread.detach();

  }

  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;

  void send_goal(double x, double y, double theta)
  {
    if (!action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    cancel_sent_ = false;
    goal_active_ = true;

    auto goal_msg = Goto::Goal();
    goal_msg.x_position = x;
    goal_msg.y_position = y;
    goal_msg.theta_position = theta;

    using namespace std::placeholders;

    rclcpp_action::Client<Goto>::SendGoalOptions options;

    options.goal_response_callback =
      std::bind(&GoToClient::goal_response_callback, this, _1);

    options.feedback_callback =
      std::bind(&GoToClient::feedback_callback, this, _1, _2);

    options.result_callback =
      std::bind(&GoToClient::result_callback, this, _1);

    action_client_->async_send_goal(goal_msg, options);

  }

private:
  rclcpp_action::Client<Goto>::SharedPtr action_client_;
  GoalHandleGoto::SharedPtr goal_handle_;
  std::thread input_thread;
  bool cancel_sent_;
  double target_x, target_y, target_theta;
  bool goal_active_ = false;

  void flush_stdin() {
    tcflush(STDIN_FILENO, TCIFLUSH);
    std::cin.clear();
  }

  void goal_response_callback(GoalHandleGoto::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Goal rejected");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    goal_handle_ = goal_handle;
  }

  void feedback_callback(GoalHandleGoto::SharedPtr, const std::shared_ptr<const Goto::Feedback> feedback)
  {
    double x_remaining = feedback->x_distance;
    double y_remaining = feedback->y_distance;
    double orientation = feedback->orientation;

    RCLCPP_INFO(this->get_logger(),
                "Feedback: remaining x_distance = %f, remaining y_distance = %f, remaining rotation = %f",
                x_remaining, y_remaining, orientation);

    if (cancel_sent_ || !goal_handle_) {
      return;
    }

    const double dist_threshold = 1.0; 
    const double angle_threshold = 0.1;

    if (std::abs(x_remaining) < dist_threshold && std::abs(y_remaining) < dist_threshold && std::abs(orientation) < angle_threshold){
      cancel_sent_ = true;
      RCLCPP_WARN(this->get_logger(),
                  "Remaining distance too small, cancelling goal...");

      action_client_->async_cancel_goal(goal_handle_);
    }
  }

  void result_callback(const GoalHandleGoto::WrappedResult & result)
  {

    if (!result.result) {
    RCLCPP_ERROR(this->get_logger(), "No final results");
    return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Result status=%d, x_end=%f, y_end=%f, theta_end=%f\nSend any value to give a new goal:",
                result.code,
                result.result->x_end_position,
                result.result->y_end_position,
                result.result->end_orientation);

    this->goal_handle_ = nullptr;
    goal_active_ = false; 
  }

  void user_input_loop() {
    while (rclcpp::ok()) {

      if (goal_active_) {
        std::string input;
        if (std::cin >> input) {
          if ((input == "c" || input == "C") && goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "Cancel request sent...");
            action_client_->async_cancel_goal(goal_handle_);
            while(goal_active_ && rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      
        continue;

      }

      while (rclcpp::ok() && !goal_active_) {
        std::cout << "Insert X coordinate: " << std::flush;
        if (std::cin >> target_x) break;
        std::cout << "Error: insert a number." << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }

      while (rclcpp::ok() && !goal_active_) {
        std::cout << "Insert Y coordinate: " << std::flush;
        if (std::cin >> target_y) break;
        std::cout << "Error: insert a number." << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }

      while (rclcpp::ok() && !goal_active_) {
        std::cout << "Insert Theta (radians): " << std::flush;
        if (std::cin >> target_theta) break;
        std::cout << "Error: insert a number." << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }

      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


      RCLCPP_INFO(this->get_logger(), "Sending goal: X=%f, Y=%f, Th=%f,\nINSERT 'c' OR 'C' TO CANCEL THE REQUEST", target_x, target_y, target_theta);
      this->send_goal(target_x, target_y, target_theta);

    }
  }

};

RCLCPP_COMPONENTS_REGISTER_NODE(GoToClient)