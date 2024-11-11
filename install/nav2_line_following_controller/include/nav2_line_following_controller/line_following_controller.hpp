#ifndef NAV2_LINE_FOLLOWING_CONTROLLER__LINE_FOLLOWING_CONTROLLER_HPP_
#define NAV2_LINE_FOLLOWING_CONTROLLER__LINE_FOLLOWING_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "route.h"

namespace nav2_line_following_controller
{

class LineFollowingController : public nav2_core::Controller
{
public:
  LineFollowingController() = default;
  ~LineFollowingController() override = default;



  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, 
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf, 
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    );

  rcl_interfaces::msg::SetParametersResult on_parameters_set_callback(
    const std::vector<rclcpp::Parameter> &);
  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:


  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("LineFollowingController")};
  rclcpp::Clock::SharedPtr clock_;

  double max_cte_;
  double max_velocity_;
  double max_reverse_velocity_;
  double max_acceleration_;
  double max_deceleration_;
  double max_lateral_acceleration_;
  double lookahead_distance_;
  double min_turn_radius_;
  double route_smoothing_strength_;
  double steering_k_d_;
  double steering_k_p_;

  std::vector<std::shared_ptr<Route>> routes_;
  size_t current_subroute_index_ = 0;
  std::shared_ptr<Route> route_;
  std::shared_ptr<Route::Position> route_position_;

  rclcpp::Duration transform_tolerance_ {0, 0};

  nav_msgs::msg::Path global_plan_;

  void publish_local_plan();

  // desired velocity for each point in global_plan_
  std::vector<double> velocity_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> lookahead_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_;


};

}  // namespace nav2_line_following_controller

#endif  // NAV2_LINE_FOLLOWING_CONTROLLER__LINE_FOLLOWING_CONTROLLER_HPP_