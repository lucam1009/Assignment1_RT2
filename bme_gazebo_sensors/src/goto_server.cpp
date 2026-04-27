#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/ass1goto.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2/utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GoToServer : public rclcpp::Node
{
public:
  using Goto = action_interfaces::action::Ass1goto;
  using GoalHandleGoto = rclcpp_action::ServerGoalHandle<Goto>;


  explicit GoToServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("goto_server", options)
  {
    using namespace std::placeholders;

    sub_position = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&GoToServer::callback_position, this, _1));

    pub_velocity = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->action_server_ = rclcpp_action::create_server<Goto>(
      this,
      "goto",
      std::bind(&GoToServer::handle_goal, this, _1, _2),
      std::bind(&GoToServer::handle_cancel, this, _1),
      std::bind(&GoToServer::handle_accepted, this, _1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  rclcpp_action::Server<Goto>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_position;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double current_x = 0.0;
  double current_y = 0.0;
  double current_theta = 0.0;

  void callback_position(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    current_theta = tf2::getYaw(msg->pose.pose.orientation);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";

    t.transform.translation.x = current_x;
    t.transform.translation.y = current_y;
    t.transform.translation.z = 0.0;

    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goto::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGoto> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoto> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GoToServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoto> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Goto::Feedback>();
    auto result = std::make_shared<Goto::Result>();

    geometry_msgs::msg::TransformStamped goal_frame_world;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

  
    while(rclcpp::ok()){  

      goal_frame_world.header.stamp = this->get_clock()->now();
    goal_frame_world.header.frame_id = "world";
    goal_frame_world.child_frame_id = "goal";
    goal_frame_world.transform.translation.x = goal->x_position;
    goal_frame_world.transform.translation.y = goal->y_position;
    goal_frame_world.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal->theta_position);
    goal_frame_world.transform.rotation = tf2::toMsg(q);
      
    tf_broadcaster_->sendTransform(goal_frame_world);


      geometry_msgs::msg::Twist message;

      if (goal_handle->is_canceling()) {
        result->x_end_position = current_x;
        result->y_end_position = current_y;
        result->end_orientation = current_theta;
        goal_handle->canceled(result);
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.angular.z = 0.0;
        pub_velocity->publish(message);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }    

      geometry_msgs::msg::TransformStamped goal_frame_robot;

      try{
        goal_frame_robot = tf_buffer_->lookupTransform("base_link","goal", tf2::TimePointZero,tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform goal frame to robot frame: %s",
            ex.what());
          continue;
      }

      double dx = goal_frame_robot.transform.translation.x;
      double dy = goal_frame_robot.transform.translation.y;
      double local_theta = tf2::getYaw(goal_frame_robot.transform.rotation);

      double distance = sqrt(pow(dx, 2) + pow(dy, 2));
      double angle_error = atan2(dy, dx);

      if (distance > 0.1) {
        message.linear.x = std::min(0.6, 0.5 * distance); 
        message.angular.z = 1.2 * angle_error; 
        pub_velocity->publish(message);

      } else {
        message.linear.x = 0.0;

        double angle_to_target = atan2(sin(local_theta), cos(local_theta));

        if (std::abs(angle_to_target) > 0.02) { 
          message.angular.z = 1.0 * angle_to_target;
          if (message.angular.z > 0.8) 
            message.angular.z = 0.8;
          if (message.angular.z < -0.8) 
            message.angular.z = -0.8;
          pub_velocity->publish(message);
        } else {
          message.angular.z = 0.0;
          pub_velocity->publish(message);
          RCLCPP_INFO(this->get_logger(), "Final orientation reached");
          break;
        }
      }

      /*feedback->x_distance = dx;
      feedback->y_distance = dy;
      feedback->orientation = angle_to_target;
      goal_handle->publish_feedback(feedback);*/
      loop_rate.sleep();

    }


    if (rclcpp::ok()) {
      result->x_end_position = current_x;
      result->y_end_position = current_y;
      result->end_orientation = current_theta;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};


RCLCPP_COMPONENTS_REGISTER_NODE(GoToServer)